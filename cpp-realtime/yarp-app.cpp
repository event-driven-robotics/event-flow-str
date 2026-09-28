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
    int con_min{20};
    double trip_tol{0.125};
    int smooth{3};
    double period{0.03};
    cv::Size img_size, out_img_size;

    ev::window<ev::AE> input;
    std::thread vt, et;
    ev::info inf; 
    double packets{0.0}, proc_time{0.0}, rate{0.0};
    std::string filename;

    zrtFlow zrt_flow;
    cv::Mat sample, sample_sparse, canvas;

    cv::VideoCapture webcam;
    bool webcam_opened{false};
    cv::VideoWriter saver;
    std::list<cv::Mat> frames;
    
    static constexpr double alpha{0.3};
    inline static const cv::Vec3b white{255, 255, 255};
    inline static const cv::Vec3b blue{255, 0, 0};
    inline static const cv::Vec3b orange{0, 128, 255};   

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        if(rf.check("help") || rf.check("h")) {
            yInfo() << "Usage: yarp-flow-str [options]\n"
                    << "  --name <string>            Module name (default: /event-flow-str)\n"
                    << "  --output_video <string>    Output video file (default: disabled)\n"
                    << "  --block_size <pixels>      Block size:  (default: 40, advise cam width/32)\n"
                    << "  --max_n <events>           Maximum events (default: 80, higher = slower)\n"
                    << "  --con_d <pixels>           Connectivity distance (default: 2, triplet search region)\n"
                    << "  --con_min <triplets>       Connectivity minimum (default: 20)\n"
                    << "  --trip_tol <seconds>       Triplet tolerance (default: 0.125)\n"
                    << "  --smooth <blocks>          Smoothing (default: 3)\n"
                    << "  --width <pixels>           Image width (default: 640)\n"
                    << "  --height <pixels>          Image height (default: 480)\n"
                    << "  --period <seconds>         Update period in seconds (default: 0.03)";
            return false;
        }
        setName(rf.check("name", Value("/event-flow-str")).asString().c_str());
        
        if(!Network::checkNetwork(1.0)) {
            yError() << "could not connect to YARP";
            return false;
        }

        if(!input.open(getName() +"/AE:i")) {
            yError() << "could not open input port";
            return false;
        }

        block_size = rf.check("block_size", Value(block_size)).asInt32();
        max_n = rf.check("max_n", Value(max_n)).asInt32();
        con_d = rf.check("con_d", Value(con_d)).asInt32();
        con_min = rf.check("con_min", Value(con_min)).asInt32();
        trip_tol = rf.check("trip_tol", Value(trip_tol)).asFloat64();
        smooth = rf.check("smooth", Value(smooth)).asInt32();

        int width = rf.check("width", Value(640)).asInt32();
        int height = rf.check("height", Value(480)).asInt32();
        img_size = {width, height};

        period = rf.check("period", Value(0.03)).asFloat64();

        webcam_opened = webcam.open(-1);
        if (!webcam.isOpened()) {
            yInfo() << "No webcam found, just running the flow visualisation";
            out_img_size = img_size;
        } else {
            yInfo() << "Webcam opened, running the flow visualisation with webcam feed";
            webcam.set(cv::CAP_PROP_FRAME_HEIGHT, height / 2); //hack to get webcam to work?
            webcam.set(cv::CAP_PROP_FRAME_WIDTH, width / 2); 
            yInfo() << webcam.get(cv::CAP_PROP_FRAME_HEIGHT) << webcam.get(cv::CAP_PROP_FRAME_WIDTH);
            out_img_size = {width * 2, height};
        }

        // BLOCK, n, d, update#, max_dt, tolerance, SMOOTH
        zrt_flow.initialise(img_size, block_size, max_n, con_d, con_min, trip_tol, smooth);
        sample = cv::Mat(img_size, CV_8UC3);
        sample_sparse = cv::Mat(img_size, CV_8UC3, cv::Vec3b(255, 255, 255));
        canvas = cv::Mat(out_img_size, CV_8UC3);

        filename = rf.check("output_video", Value("")).asString();
        if(!filename.empty()) {
            saver.open(filename,
                   cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                   1.0 / getPeriod(), out_img_size, true);
            if (!saver.isOpened()) {
                yError() << "Could not open output video at" << filename;
                return false;
            }
        }

        cv::namedWindow("EVENT-FLOW-STR", cv::WINDOW_NORMAL);
        cv::resizeWindow("EVENT-FLOW-STR", out_img_size);

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
        while (input.isRunning()) {
            auto this_inf = input.readAll(true);
            if(!input.isRunning()) break;

            inf.duration += this_inf.duration * alpha; //alpha is a smoothing factor for visualisation

            double tic = yarp::os::Time::now();
            //for each event add it to the block
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
        while (input.isRunning()) {
            double tic = yarp::os::Time::now();
            zrt_flow.update();
            rate = ((yarp::os::Time::now() - tic) + rate) * 0.5; //0.5 is a smoothing factor for visualisation
            zrt_flow.makebgr().copyTo(sample);
        }
    }


    void drawBars(cv::Mat img) {
        static int bar_h = img_size.height / 2 - 20;

        int pix_rate = (int)(bar_h * (inf.count / inf.duration) / 50.0e6);
        cv::rectangle(img, {10, 10 + bar_h - pix_rate, 20, pix_rate}, blue, cv::FILLED);
        int pix_late = (int)(bar_h * proc_time / (packets * 0.005));  // 0.01 is 10 ms
        cv::rectangle(img, {10, img_size.height / 2 + 10 + bar_h - pix_late, 20, pix_late}, orange, cv::FILLED);

        cv::rectangle(img, {10, 10, 20, bar_h}, {0, 0, 0}, 2);
        cv::putText(img, "50M ev/s", {35, 30}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        cv::putText(img, "Event Rate", {35, img_size.height / 2 - 10}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        for (int i = 10; i < bar_h; i += bar_h / 5)
            cv::line(img, {10, i}, {30, i}, {0, 0, 0});

        cv::rectangle(img, {10, img_size.height / 2 + 10, 20, bar_h}, {0, 0, 0}, 2);
        cv::putText(img, "5 ms", {35, img_size.height / 2 + 30}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        cv::putText(img, "Latency", {35, img_size.height - 10}, cv::FONT_HERSHEY_PLAIN, 2.0, {0, 0, 0}, 2, cv::LINE_AA, false);
        for (int i = img_size.height / 2 + 10; i < img_size.height / 2 + 10 + bar_h; i += bar_h / 5)
            cv::line(img, {10, i}, {30, i}, {0, 0, 0});
    }

    bool updateModule() override {
        static cv::Mat wc;
        if(webcam_opened) {
            webcam >> wc;
            cv::resize(wc, canvas({img_size.width, 0, img_size.width, img_size.height}), img_size);
        }

        sample_sparse.copyTo(canvas({{0, 0}, img_size}));
        sample_sparse = white;
        
        drawBars(canvas);

        if(!filename.empty()) {
            frames.emplace_back(cv::Mat());
            canvas.copyTo(frames.back());
        }
        cv::imshow("EVENT-FLOW-STR", canvas);
        cv::waitKey(1);

        yInfo() << int(inf.duration * 1.0e6 / packets) << "us|" << int(1.0 / rate) << "hz|" << packets << " packets|" << inf.count / inf.duration << "events/second";
        inf.duration *= (1-alpha);
        inf.count *= (1-alpha);
        packets *= (1-alpha);
        proc_time *= (1-alpha);

        return inf.timestamp;  //::os::Time::now();//inf.timestamp;
    }

    

    bool interruptModule() override
    {
        //causes updateModule to return false, close should be called
        return true;
    }

    bool close() override {
        if(!filename.empty()) {
            for (auto& f : frames)
                saver << f;
            saver.release();
            yInfo() << "Saved video to" << filename;
        }
        yInfo() << "Stopping input port";
        input.stop();
        yInfo() << "Waiting for flow thread";
        vt.join();
        yInfo() << "Waiting for event thread";
        et.join();
        yInfo() << "Release webcam";
        if(webcam_opened) webcam.release();
        return true;
    }
};

int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.configure(argc, argv);

    yarp_flow_app yarpapp;
    return yarpapp.runModule(rf);
}
