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

#pragma once
#include <chrono>
#include <opencv2/opencv.hpp>

class zrtBlock {
    friend class zrtFlow;
private:
    int N{0};         //this is the maximum number of events to update
    cv::Point2d flow; //raw flow assigned to this block
    std::deque<double> x_dist; //distribution of x connections
    std::deque<double> y_dist; //distribution of y connections

    std::vector<cv::Point> pxs_live, pxs_snap; //live/snap circular buffer
    int i{0}, is{0}; //new event position live/snap
    int j{0}, js{0}; //previously updated position live/snap
    
    std::chrono::steady_clock::time_point last_update_tic{std::chrono::steady_clock::now()};

    //calculate connections for a single event on the SAE
    void singlePixConnections(cv::Mat &sae, int d, double triplet_tolerance, cv::Point p0);

public:

    zrtBlock(int N);

    //add a new point to the block
    void add(cv::Point p);

    //snap saves a copy of the live block into the "snap" variables
    void snap();

    //update connections for each new event
    void updateConnections(cv::Mat &sae, int d, double triplet_tolerance);

    //udate the flow state from the connection buffer
    void updateFlow(size_t n = 0);

};

class zrtFlow
{
private:

    enum {X=0,Y=1};
 
    cv::Mat sae;
    std::vector<zrtBlock> blocklist;
    std::vector<zrtBlock*> blockmap;
    cv::Size array_dims{{0, 0}};
    cv::Size block_dims{{0, 0}};
    cv::Size image_res{{0, 0}};

    cv::Mat block_flow[2];
    cv::Mat pixel_flow[2];
    cv::Mat full_flow[2];
    cv::Mat hsv, rgb;

    //parameters
    int con_len{3};
    double trip_tol{0.125};
    int con_buf_min{20};
    int smooth_factor{3};

public:

    void initialise(cv::Size res, int block_size, int max_N, int connection_length, int con_buf_min, double trip_tol, int smooth_factor);
    
    //add a new event to the SAE and record the new event with the
    //corresponding block
    void add(int u, int v, double t);

    //go through each block and update the list of flow vectors
    //update the final flow per pixel
    void update();

    cv::Mat makebgr();
};