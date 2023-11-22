/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <System.h>

using namespace std;

bool b_continue_session = true;

void exit_loop_handler(int s)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv)
{

    if (argc < 3 || argc > 5)
    {
        cerr << endl
             << "Usage: ./mono_webcam [path_to_vocabulary] [path_to_settings] [path_to_video_file] (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 5)
    {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }


    string video_file;

    video_file = string(argv[3]);
    // Set video capture to interface 0 (default) 
    cv::VideoCapture cap(video_file);

    if (!cap.isOpened()) {
        std::cout << "Cannot open video";
        return 1;
    }


    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM("../../Vocabulary/ORBvoc.txt", "webcam.yaml", ORB_SLAM3::System::MONOCULAR, true);
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imCV;

    double t_resize = 0.f;
    double t_track = 0.f;
    int count = 0;
    int last_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cout << "LAST FRAME COUNT: " << last_frame << endl;
    while (b_continue_session)
    {
        double timestamp_ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
        cap >> imCV;
        if(!imCV.empty()){
        /*if (imCV.empty()){
            break;
        }*/
        // cv::imshow("frame", imCV);
        // cv::waitKey(0);
        //cv::rotate(imCV, imCV, cv::ROTATE_90_CLOCKWISE);
        //cv::Mat gray;
        //cv::cvtColor(imCV, gray, cv::COLOR_BGR2GRAY);
        imageScale = 1.0f;
        // if (imageScale != 1.f)
        // {
        //     int width = imCV.cols * imageScale;
        //     int height = imCV.rows * imageScale;
        //     cv::resize(imCV, imCV, cv::Size(width, height));
        // }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imCV, timestamp_ms);
        
        // if(count >= last_frame) {
        //     break;
        // }
        // else {
        //     count++;
        // }
        }
    }

    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryEuRoC("testfahrt.txt");

    return 0;
}