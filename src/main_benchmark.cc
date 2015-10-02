/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "Converter.h"


using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;

    // -------------------------------------------------------------------------
    // Load Settings and Camera
    std::string dataset_dir = ros::package::getPath("rpg_datasets");
    std::string dataset_name, trace_dir;
    ros::param::get("~dataset_name", dataset_name);
    ros::param::get("~trace_dir", trace_dir);
    std::string camSettingsFile = dataset_dir + "/" + dataset_name + "/orb_calib.yaml";
    std::string orbSettingsFile = ros::package::getPath("ORB_SLAM")+"/Data/Settings.yaml";
    cv::FileStorage orbSettings(orbSettingsFile.c_str(), cv::FileStorage::READ);
    if(!orbSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    // -------------------------------------------------------------------------
    // Setup ORB SLAM

    // Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    // New version to load vocabulary from text file "Data/ORBvoc.txt". 
    // If you have an own .yml vocabulary, use the function
    // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
    string strVocFile = ros::package::getPath("ORB_SLAM") + "/Data/ORBvoc.txt";
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    
    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        ros::shutdown();
        return 1;
    }

    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    // Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    // Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    // Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(
          &Vocabulary, &FramePub, &MapPub, &World, camSettingsFile, orbSettingsFile);
    // boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);
    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);
    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);
    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    //This "main" thread will show the current processed frame and publish the map
    float fps = orbSettings["Camera.fps"];
    if(fps==0)
      fps=30;

    //--------------------------------------------------------------------------
    // Load dataset

    std::string img_filenames = dataset_dir + "/" + dataset_name + "/data/images.txt";
    std::ifstream img_fs(img_filenames.c_str());
    if(!img_fs.is_open())
    {
      std::cout << "Could not open images file " << img_filenames << std::endl;
      return 0;
    }

    ros::Rate r(fps);
    int first_frame_id, last_frame_id;
    ros::param::get("~dataset_first_frame", first_frame_id);
    ros::param::get("~dataset_last_frame", last_frame_id);
    while(img_fs.good() && !img_fs.eof() && ros::ok())
    {
      if(img_fs.peek() == '#') // skip comments
        img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      //--------------------------------------------------------------------------
      // Load image
      int img_id;
      double stamp_seconds;
      std::string img_name;
      img_fs >> img_id >> stamp_seconds >> img_name;
      img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      if(img_id < first_frame_id)
        continue;
      else if(img_id > last_frame_id)
        break;
      std::string img_filename(dataset_dir + "/" + dataset_name + "/data/" + img_name);
      cv::Mat img = cv::imread(img_filename, 0);
      if(img.empty())
      {
        std::cout << "Image empty:" << img_filename << std::endl;
        break;
      }


      //--------------------------------------------------------------------------
      // Track pose
      Tracker.ProcessImage(img, img_id);
      FramePub.Refresh();
      MapPub.Refresh();
      Tracker.CheckResetByPublishers();
      r.sleep();
    }

    // ---------------------------------------------------------------------------
    // Trace poses to file
    std::string trace_pose_filename = trace_dir + "/traj_estimate.txt";
    std::ofstream trace_pose(trace_pose_filename.c_str());
    assert(!trace_pose.fail());
    std::vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM::KeyFrame::lId);
    std::cout << "Saving " << vpKFs.size() << " poses..." << std::endl;
    for(size_t i = 0; i < vpKFs.size(); ++i)
    {
      ORB_SLAM::KeyFrame* pKF = vpKFs[i];
      if(pKF->isBad())
        continue;

      cv::Mat R = pKF->GetRotation().t();
      cv::Mat t = pKF->GetCameraCenter();
      vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
      trace_pose << static_cast<int>(pKF->mTimeStamp) << " "
                 << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
                 << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
                 << std::endl;
    }
    std::cout << "...done." << std::endl;

    ros::shutdown();
    return 0;
}
