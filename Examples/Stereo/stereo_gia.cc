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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

namespace fs = std::filesystem;

std::string getCurrentTimeString() {
    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm* now_tm = std::localtime(&now_time);

    // Format the time as YYMMDDHHMMSS
    std::stringstream ss;
    ss << std::put_time(now_tm, "%y%m%d_%H%M%S");

    return ss.str();
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, const int& start_id);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./stereo_gia path_to_vocabulary path_to_settings path_to_sequence start_id eval_log_dir" << endl;
        return 1;
    }

    int start_id = std::stoi(argv[4]);
    // the first 50 frames will not be used
    if(start_id < 51){
        cerr << "the first 50 frames should not be used because of the sensor synchronisation problem" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps, start_id);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    std::string current_time = getCurrentTimeString();
    
    fs::path eval_log_dir = string(argv[5]);
    eval_log_dir = eval_log_dir / current_time;
    try {
        // Check if the directory exists
        if (!fs::exists(eval_log_dir)) {
            // Create the directory
            if (!fs::create_directory(eval_log_dir)) {
               std::cout << "Failed to create the evaluation directory." << std::endl;
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    try {
        // Define the source file and the destination folder
        fs::path yaml_source_file = std::filesystem::current_path() / string(argv[2]);
        fs::path destination_folder = eval_log_dir;

        // Construct the full path for the destination file
        fs::path destination_file = destination_folder / yaml_source_file.filename();

        // Copy the file
        fs::copy(yaml_source_file, destination_file);

        // std::cout << "File copied successfully to: " << destination_file << std::endl;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // output the start_id information for evaluation
    std::ofstream ofs_id(eval_log_dir / "start_id.txt", std::ios::out);
    if (ofs_id.is_open()) {
        ofs_id << start_id << std::endl;
        ofs_id.close();
    }

    // output the tracking times for evaluation
    std::ofstream ofs(eval_log_dir / "track_times.txt", std::ios::out);
    if (ofs.is_open()) {
        for (const auto track_time : vTimesTrack) {
            ofs << track_time << std::endl;
        }
        ofs.close();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI(eval_log_dir / "frame_trajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, const int& start_id)
{
    ifstream fTimes;

    std::vector<std::string> tokens;
    std::istringstream stream(strPathToSequence);
    std::string token;

    char delimiter = '/';
    
    while (getline(stream, token, delimiter)) {
        tokens.push_back(token);
    }

    std::string dataset_record_date = tokens.at(tokens.size() - 1);

    if(dataset_record_date == ""){
        dataset_record_date = tokens.at(tokens.size() - 2);
    }

    string strPathTimeFile = strPathToSequence + "/Stereocam_" + dataset_record_date + ".log";
    std::cout << strPathTimeFile << std::endl;
    fTimes.open(strPathTimeFile.c_str());
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence;
    string strPrefixRight = strPathToSequence;

    vTimestamps.erase(vTimestamps.begin(), vTimestamps.begin() + start_id);

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i + start_id;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + "L.jpg";
        vstrImageRight[i] = strPrefixRight + ss.str() + "R.jpg";
    }
}
