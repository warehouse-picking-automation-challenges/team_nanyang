#ifndef _VIRTUAL_KINECT_H_
#define _VIRTUAL_KINECT_H_

#pragma once

#if (defined WIN32 || defined _WIN32 || defined WINCE) // for Windows
#include <Windows.h>
#include <direct.h>
#include <io.h>
#else   // for Linux
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstddef>
#endif

#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

#define VK_CAP_TYPE_DEPTH    CV_CAP_OPENNI_DEPTH_MAP
#define VK_CAP_TYPE_IMAGE    CV_CAP_OPENNI_BGR_IMAGE
#define VK_CAP_TYPE_POINT    CV_CAP_OPENNI_POINT_CLOUD_MAP

#define VK_DEPTH_FILE_UNKOWN 0
#define VK_DEPTH_FILE_VIDEO  2
#define VK_DEPTH_FILE_IMAGE  4

#define VK_COLOR_FILE_UNKOWN 0
#define VK_COLOR_FILE_VIDEO  2
#define VK_COLOR_FILE_IMAGE  4

#define DEFAULT_DEPTH_FOV_H  CV_PI*58.0/180.0
#define DEFAULT_DEPTH_FOV_V  CV_PI*45.0/180.0

class VirtualKinect
{
public:
    typedef int VK_DEPTH_FILE_TYPE;
    typedef int VK_COLOR_FILE_TYPE;

protected:
    std::string dataFolder_;
    int frameCount_;
    int framePos_;
    std::string tCreated_;
    bool bOpened_;

    bool depth_16uc1_8uc3(const cv::Mat& depth, cv::Mat& depthImg);

    bool depth_8uc3_16uc1(const cv::Mat& depthImg, cv::Mat& depth);

    void showError(const std::string err);

    void showVideoInfo(cv::VideoCapture& video);

    void mapProjectiveToRealWorld(cv::Point3f& src, cv::Point3f& dst);

public:
    VirtualKinect()
        : frameCount_(0), framePos_(0), 
        bOpened_(false) { tCreated_ = getCurrTimeString(); }

    VirtualKinect(std::string dataFolder)
        : dataFolder_(dataFolder), frameCount_(0), framePos_(0), 
        bOpened_(false)
    {
        tCreated_ = getCurrTimeString();
        open(dataFolder_);
    }

    ~VirtualKinect() {}

    virtual bool open(std::string dataFolder)
    {
        bOpened_ = isFolderExist(dataFolder_);
        return bOpened_;
    }

    bool isOpened() { return bOpened_; }

    bool generatePointClouds(const cv::Mat& depth, cv::Mat& points);

    bool averagingDepthMaps(std::vector<cv::Mat>& multiDepths, cv::Mat& depth);

    bool isFolderExist(const std::string folder, bool creatFolder = false );

    bool isFileReadable(const std::string file);

    std::string getCurrTimeString();

};

class VirtualKinectWriter : public VirtualKinect
{
private:
    cv::VideoWriter imageWriter_;
    cv::VideoWriter depthWriter_;
    VK_DEPTH_FILE_TYPE depthFileType_;
    VK_COLOR_FILE_TYPE colorFileType_;
    std::ofstream ofs_files_;

public:
    VirtualKinectWriter() : VirtualKinect() {};
    VirtualKinectWriter(std::string dataFolder, VK_DEPTH_FILE_TYPE typeDepth, VK_COLOR_FILE_TYPE typeColor)
        : VirtualKinect(dataFolder), depthFileType_(typeDepth), colorFileType_(typeColor) { open(dataFolder_, depthFileType_, colorFileType_); }
    virtual ~VirtualKinectWriter()
    {
        if (ofs_files_.is_open())
        {
            ofs_files_.close();
        }
    }

    virtual bool open(std::string dataFolder, VK_DEPTH_FILE_TYPE typeDepth, VK_COLOR_FILE_TYPE typeColor);

    bool write(cv::Mat& depth);
    bool write(cv::Mat& depth, cv::Mat& image);
};

class VirtualKinectCapture : public VirtualKinect
{
private:
    VK_DEPTH_FILE_TYPE depthFileType_;
    VK_COLOR_FILE_TYPE colorFileType_;
    std::vector<std::string> colorFile_;
    std::vector<std::string> depthFile_;
    cv::Mat image_;
    cv::Mat depth_;
    cv::Mat point_;
    cv::VideoCapture colorCapture_;
    cv::VideoCapture depthCapture_;
    int dID_;

    int findSubString(std::string S, std::string T, int pos = 0);

    bool parseFileList(const std::string vkDatas);

public:
    VirtualKinectCapture() 
        : VirtualKinect(), depthFileType_(VK_DEPTH_FILE_UNKOWN), colorFileType_(VK_COLOR_FILE_UNKOWN), dID_(0) {}
    VirtualKinectCapture(std::string dataList) 
        : VirtualKinect(), depthFileType_(VK_DEPTH_FILE_UNKOWN), colorFileType_(VK_COLOR_FILE_UNKOWN), dID_(0) { open(dataList); }
    virtual ~VirtualKinectCapture() 
    { 
        release(); 
    }

    virtual bool open(std::string filelist);

    void release();

    bool grab();
    bool grab(int frameID);

    bool retrieve(cv::Mat& frame, int flag);

};


#endif
