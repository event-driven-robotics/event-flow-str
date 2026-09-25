/*
 *   Copyright (C) 2026 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it; zhichao.li@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <mutex>
#include <sstream>
#include <thread> 

#include <yarp/os/all.h>
using namespace yarp::os;

#include <event-driven/core.h>
#include "flow.h"

class yarp_flow_app : public yarp::os::RFModule 
{
private:

    int block_size{40};
    int max_n{80};
    int con_d{2};
    int con_upd{20};
    double trip_tol{0.125};
    int smooth{3};
    double rate{0.0};

    std::thread vt, et;
    ev::info inf; double packets{0.0}; double proc_time{0.0};

    zrtFlow zrt_flow;
    cv::Mat sample, sample_sparse, canvas;

    cv::VideoCapture webcam;
    cv::VideoWriter saver;
    std::list<cv::Mat> frames;
    cv::Size img_size;
    double period;
    ev::window<ev::AE> input;
    double alpha{0.3};
    inline static const cv::Vec3b white{255, 255, 255};
    inline static const cv::Vec3b blue{255, 0, 0};
    inline static const cv::Vec3b orange{0, 128, 255};   

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        setName(rf.check("name", Value("/event-flow-str")).asString().c_str());
        
        if(!Network::checkNetwork(1.0)) {
            yError() << "could not connect to YARP";
            return false;
        }

        if(!input.open(rf.check("input", Value(getName() +"/AE:i")).asString())) {
            yError() << "could not open input port";
            return false;
        }

        int width = rf.check("width", Value(640)).asInt32();
        int height = rf.check("height", Value(480)).asInt32();
        img_size = {width, height};

        period = rf.check("period", Value(0.33)).asFloat64();

        // BLOCK, n, d, update#, max_dt, tolerance, SMOOTH
        zrt_flow.initialise(img_size, block_size, max_n, con_d, con_upd, trip_tol, smooth);
        sample = cv::Mat(img_size, CV_8UC3);
        sample_sparse = cv::Mat(img_size, CV_8UC3, cv::Vec3b(255, 255, 255));
        canvas = cv::Mat(img_size, CV_8UC3);

        
        webcam.open(-1);
        if (!webcam.isOpened()) {
            yError() << "Could not open webcam";
            return false;
        }
        webcam.set(cv::CAP_PROP_FRAME_HEIGHT, height / 2); //hack?
        webcam.set(cv::CAP_PROP_FRAME_WIDTH, width / 2); //hack?

        saver.open("/home/aglover-iit.local/Downloads/flow_demo.mp4",
                   cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                   1.0 / getPeriod(), {width * 2, height}, true);
        if (!saver.isOpened()) {
            yError() << "Could not open output video";
            return false;
        }
        yInfo() << webcam.get(cv::CAP_PROP_FRAME_HEIGHT) << webcam.get(cv::CAP_PROP_FRAME_WIDTH);

        cv::namedWindow("EVENT-FLOW-STR", cv::WINDOW_NORMAL);
        cv::resizeWindow("EVENT-FLOW-STR", img_size);

        vt = std::thread([this] { updateFlowBuffer(); });
        et = std::thread([this] { updateEvents(); });

        return true;
    }

    double getPeriod() override
    {
        return period;
    }

    //add events to each block
    void updateEvents() {
        while (true) {
            auto this_inf = input.readAll(true);

            inf.duration += this_inf.duration * alpha;

            double tic = yarp::os::Time::now();
            for (auto v = input.begin(); v != input.end(); v++) {
                zrt_flow.add(v->x, v->y, v.timestamp());
                sample_sparse.at<cv::Vec3b>(v->y, v->x) = white - sample.at<cv::Vec3b>(v->y, v->x);
            }
            proc_time += (yarp::os::Time::now() - tic) * alpha;
            inf.count += this_inf.count * alpha;
            packets += 1 * alpha;
            inf.timestamp = this_inf.timestamp;
        }
    }

    //update all the blocks
    void updateFlowBuffer() {
        input.readPacket(true);
        while (input.isRunning()) {
            double tic = yarp::os::Time::now();
            zrt_flow.update();
            rate = ((yarp::os::Time::now() - tic) + rate) * 0.5;
            zrt_flow.makebgr().copyTo(sample);
        }
    }



    bool updateModule() override {
        static cv::Mat wc;
        webcam >> wc;
        //cv::resize(wc, canvas({img_size.width, 0, img_size.width, img_size.height}), img_size);

        sample_sparse.copyTo(canvas({{0, 0}, img_size}));
        sample_sparse = white;
        

        static int bar_h = img_size.height / 2 - 20;

        int pix_rate = (int)(bar_h * (inf.count / inf.duration) / 50.0e6);
        cv::rectangle(canvas, {10, 10 + bar_h - pix_rate, 20, pix_rate}, blue, cv::FILLED);
        int pix_late = (int)(bar_h * proc_time / (packets * 0.005));  // 0.01 is 10 ms
        cv::rectangle(canvas, {10, img_size.height / 2 + 10 + bar_h - pix_late, 20, pix_late}, orange, cv::FILLED);

        cv::rectangle(canvas, {10, 10, 20, bar_h}, {0, 0, 0}, 2);
        cv::putText(canvas, "50M ev/s", {35, 30}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        cv::putText(canvas, "Event Rate", {35, img_size.height / 2 - 10}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        for (int i = 10; i < bar_h; i += bar_h / 5)
            cv::line(canvas, {10, i}, {30, i}, {0, 0, 0});

        cv::rectangle(canvas, {10, img_size.height / 2 + 10, 20, bar_h}, {0, 0, 0}, 2);
        cv::putText(canvas, "5 ms", {35, img_size.height / 2 + 30}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        cv::putText(canvas, "Latency", {35, img_size.height - 10}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        for (int i = img_size.height / 2 + 10; i < img_size.height / 2 + 10 + bar_h; i += bar_h / 5)
            cv::line(canvas, {10, i}, {30, i}, {0, 0, 0});

        frames.emplace_back(cv::Mat());
        canvas.copyTo(frames.back());
        cv::imshow("EVENT-FLOW-STR", canvas);
        cv::waitKey(1);

        yInfo() << int(inf.duration * 1.0e6 / packets) << "us|" << int(1.0 / rate) << "hz|" << packets << " packets|" << inf.count / inf.duration << "events/second";
        inf.duration *= 0.7;
        inf.count *= 0.7;
        packets *= 0.7;
        proc_time *= 0.7;

        return inf.timestamp;  //::os::Time::now();//inf.timestamp;
    }

    

    bool interruptModule() override
    {
        yInfo() << "interrupt caught";
        return true;
    }

    bool close() override {
        yInfo() << "saving frames";
        for (auto& f : frames)
            saver << f;
        saver.release();
        input.stop();
        vt.join();
        return true;
    }
};

int main(int argc, char* argv[])
{
    yarp_flow_app yarpapp;
    ResourceFinder rf;
    //rf.setDefaultConfigFile("/usr/local/src/EDOPT/configCAR.ini");
    rf.configure(argc, argv);
    
    return yarpapp.runModule(rf);
}
