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

#include "flow.h"
#include <vector>

zrtBlock::zrtBlock(int N) {
    this->N = N;
    pxs_live.resize(N);
    flow = {0.0, 0.0};
}

//i points to current point in the circular buffer to add new data
//j points to the last event not yet processed up to the maximum of the circular buffer
void zrtBlock::add(cv::Point p){
    if(++i == N) i = 0;
    pxs_live[i] = p;
    if(i == j) j++;
    if(j == N) j = 0;
}

//snap saves a copy of the live block into the "snap" variables
void zrtBlock::snap()
{
    pxs_snap = pxs_live; //snap the event circular buffer
    is = i; //snap the latest position in circular buffer
    js = j; //snap the oldest position in circular buffer
    j = i;  //set the oldest position to latest position (i.e. all data used) 

    if(is == N) is = 0; //thread "safety" if snapped during i++ command in "add"
    if(js == N) js = 0; //thread "safety" if snapped during j++ command in "add"
}

//calculate connections for a single event on the SAE
void zrtBlock::singlePixConnections(cv::Mat &sae, int d, double triplet_tolerance, cv::Point p0)
{        
    for(int dy = -d; dy <= d; dy++) {
        for(int dx = -d; dx <= d; dx++) {
            cv::Point p1 = p0 + cv::Point(dx, dy);
            cv::Point p2 = p0 + cv::Point(2*dx, 2*dy);
            if(p2.x < 0 || p2.x >= sae.size().width || p2.y < 0 || p2.y >= sae.size().height)
                continue;
            double dt12 = sae.at<double>(p0) - sae.at<double>(p1);
            double dt23 = sae.at<double>(p1) - sae.at<double>(p2);
            if(0 < dt12 && 0 < dt23) {
                double error = fabs(1 - dt23/dt12);
                if(error < triplet_tolerance) { //THRESHOLD HERE
                    double invt = 1.0 /  (dt12 + dt23);
                    x_dist.push_back(dx * invt);
                    y_dist.push_back(dy * invt);
                }
            }
        }
    }       
}

//update connections for each new event
void zrtBlock::updateConnections(cv::Mat &sae, int d, double triplet_tolerance)
{
    while (js != is) {
        js++;
        if(js == N) js = 0;
        singlePixConnections(sae, d, triplet_tolerance, pxs_snap[js]);
    }
}

//udate the flow state from the connection buffer
void zrtBlock::updateFlow(size_t n)
{
    if(n < 3) n = 3;
    if(x_dist.size() < n) {
        double magnitude = sqrt(flow.x*flow.x+flow.y*flow.y);
        double max_mag = 1.0 / (yarp::os::Time::now() - last_update_tic);
        if(magnitude > max_mag) flow *= max_mag / magnitude;
        return;
    } else {
        std::sort(x_dist.begin(), x_dist.end());
        std::sort(y_dist.begin(), y_dist.end());
        flow = {x_dist[x_dist.size()/2], y_dist[y_dist.size()/2]};
        x_dist.clear(); y_dist.clear();
        last_update_tic = yarp::os::Time::now();
    }      
}

void zrtFlow::initialise(cv::Size res, int block_size, int max_N, int connection_length, int con_buf_min, double trip_tol, int smooth_factor)
{
    //initialise the SAE
    sae = cv::Mat(res, CV_64F);

    this->con_len = connection_length;
    this->trip_tol = trip_tol;
    this->con_buf_min = con_buf_min;
    this->smooth_factor = smooth_factor;

    //calculate blocks
    block_dims = {block_size, block_size};
    array_dims = res / block_size;

    //initialise the blocks
    blocklist.resize(array_dims.area(), zrtBlock(max_N));

    //for speed initialise pointers to blocks for each pixel
    blockmap.resize(res.area());
    for(int y = 0; y < res.height; y++) {
        for(int x = 0; x < res.width; x++) {
            auto &bp = blockmap[y*res.width+x];
            int bx = x / block_dims.width;
            int by = y / block_dims.height;
            if(bx < array_dims.width && by < array_dims.height)
                bp = &blocklist[by*array_dims.width+bx];
            else
                bp = nullptr;
        }
    }

    //initialise the flow containers
    //this is the flow 1 pixel per block
    block_flow[X] = cv::Mat::zeros(array_dims, CV_32F);
    block_flow[Y] = cv::Mat::zeros(array_dims, CV_32F);

    //this is the flow at full image size
    full_flow[X] = cv::Mat::zeros(res, CV_32F);
    full_flow[Y] = cv::Mat::zeros(res, CV_32F);

    //this is the flow which might have a 0 border if blocks don't fill the full image space
    pixel_flow[X] = full_flow[X]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});
    pixel_flow[Y] = full_flow[Y]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});

}

//add a new event to the SAE and record the new event with the
//corresponding block
void zrtFlow::add(int u, int v, double t)
{
    sae.at<double>(v, u) = t;
    blockmap[v*sae.cols+u]->add({u, v});
}

//go through each block and update the list of flow vectors
//update the final flow per pixel
void zrtFlow::update()
{
    //for each block
    for(int by = 0; by < array_dims.height; by++) {
        for(int bx = 0; bx < array_dims.width; bx++) {
            
            //get the block
            auto &b = blocklist[by * array_dims.width + bx];
            //snapshot the event list so we can process in parallel
            b.snap();

            //calculate the connections for each new pixel and add to blocks flow set
            b.updateConnections(sae, con_len, trip_tol);

            //calculate the flow given the connections in
            b.updateFlow(con_buf_min);

                //asign flow to the array
            block_flow[X].at<float>(by, bx) = b.flow.x;
            block_flow[Y].at<float>(by, bx) = b.flow.y;
        }
    }
    //smooth flow - blockFilter on small image (according to zhichao)
    cv::boxFilter(block_flow[X], block_flow[X], -1, {smooth_factor, smooth_factor});
    cv::boxFilter(block_flow[Y], block_flow[Y], -1, {smooth_factor, smooth_factor});

    // cv::GaussianBlur(block_flow[X], block_flow[X], {smooth_factor, smooth_factor}, -1);
    // cv::GaussianBlur(block_flow[Y], block_flow[Y], {smooth_factor, smooth_factor}, -1);

    //resize flow - with linear interpolation (more smoothing)
    cv::resize(block_flow[X], pixel_flow[X], pixel_flow[X].size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(block_flow[Y], pixel_flow[Y], pixel_flow[Y].size(), 0, 0, cv::INTER_LINEAR);
}

cv::Mat zrtFlow::makebgr()
{
    //calculate angle and magnitude
    cv::Mat magnitude, angle;
    cv::cartToPolar(full_flow[X], full_flow[Y], magnitude, angle, true);

    //translate magnitude to range [0;1]
    cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
    magnitude *= 0.05;

    //build hsv image
    cv::Mat _hsv[3];
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    //convert to BGR
    cv::Mat rgb32;
    cv::cvtColor(hsv, rgb32, cv::COLOR_HSV2BGR);
    rgb32.convertTo(rgb, CV_8UC3, 255);
    return rgb;
}