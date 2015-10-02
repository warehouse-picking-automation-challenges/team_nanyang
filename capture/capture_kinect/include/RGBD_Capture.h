#ifndef RGBD_Capture_H
#define RGBD_Capture_H


#pragma once

#ifdef _IS_LINUX_
#include <ros/ros.h>
#include "RGBD_Capture/SRV_Capture.h"
#endif

#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "VirtualKinect.h"

#define DEFAULT_I_LOW_H 0
#define DEFAULT_I_HIGH_H 179
#define DEFAULT_I_LOW_S 0
#define DEFAULT_I_HIGH_S 255
#define DEFAULT_I_LOW_V 170
#define DEFAULT_I_HIGH_V 255
#define DEFAULT_I_NEAR_D 400
#define DEFAULT_I_FAR_D 800
#define DEFAULT_BIN_DEPTH_MM 390
#define DEFAULT_CAMERA_PITCH_ANGLE 11.0

//#ifdef _IS_WINDOWS_
#define DEBUG_MODE0
//#endif

class BinBlobsDetector
{
public:
    BinBlobsDetector();
    ~BinBlobsDetector();

    void setBinName(std::string bin_name) { bin_name_ = bin_name; }
    void setCameraBackwallDistance(float cameraBackwallDistance) { camera_backwall_distance_ = cameraBackwallDistance; }
    bool getBlobs(const cv::Mat& color, cv::Mat& cloud, std::vector<float> bin_size, int iBlobs, cv::Mat& blobs);
    Eigen::Vector3f getKinectAngle() { return kinect_angle_; }

private:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> CloudT;

    float bin_depth_;
    float bin_width_;
    float bin_height_;
    float camera_backwall_distance_;
    cv::Mat color_;
    CloudT::Ptr cloud_;
    CloudT back_wall_;
    PointT back_wall_max_pt_, back_wall_min_pt_;
    Eigen::Vector4f back_wall_coeff_;
    Eigen::Vector4f front_bar_coeff_;
    Eigen::Vector3f kinect_angle_;
    bool found_back_wall_, found_front_bar_;
    std::vector<int> mapPointToImage_;
    pcl::visualization::PCLVisualizer viewer_;   
    CloudT camera_model_;
    std::string bin_name_;

    void drawCameraModel(Eigen::Affine3f tk);    
    void init(const cv::Mat& color, cv::Mat& cloud);
    void copyCloud(cv::Mat& cvCloud, CloudT::Ptr pclCloud);
    void adjustCloud(CloudT::Ptr srcCloud, CloudT::Ptr dstCloud);
    void findBackwallAndFrontBar(std::vector<pcl::ModelCoefficients> coeffs);
    void extractBackWall();
    void voxelize(CloudT src, CloudT::Ptr dst, std::vector<std::vector<int> >& index);
    void getClusters(CloudT::Ptr cloud, std::vector<CloudT>& clusters, std::vector<std::vector<int> >& cluster_indices);
    void getClusterInBin(std::vector<CloudT>& clusters, std::vector<std::vector<int> >& cluster_indices, cv::Mat& blobs);
    void getPointsInBin(CloudT src, CloudT::Ptr dst, std::vector<std::vector<int> >& index);
};


class RGBDCapture
{
public:
    RGBDCapture(void);
    ~RGBDCapture(void);

    bool isOpened() { return bOpened_; } 
    bool retrieveImage(cv::Mat& rgbImage, cv::Mat& maskImage, int iBlobs, std::string binName);
    bool retrieveImage(cv::Mat& rgbImage, cv::Mat& depthImage, cv::Mat& maskImage, int iBlobs, std::string binName);
    bool retrieveImage(cv::Mat& rgbImage, cv::Mat& depthImage, cv::Mat& maskImage, cv::Mat& pointCloud, int iBlobs, std::string binName);
    void setMultipleGrab(int nFrames) { nMultiGrab_ = nFrames > 1 ? nFrames : 1; }
    bool averagingDepthMaps(std::vector<cv::Mat>& multiDepths, cv::Mat& depth);
    bool generateMask(const cv::Mat& color, const cv::Mat& depth, cv::Mat& dst, int iBlobs); 
    cv::Rect getROI() { return roiRect_; }
    void setBinName(std::string binname);

#ifdef _IS_LINUX_
    RGBDCapture(ros::NodeHandle& nh);
    ros::ServiceServer service;
    bool retrieve(RGBD_Capture::SRV_Capture::Request &req, RGBD_Capture::SRV_Capture::Response &res);
#endif

protected:
    bool bOpened_;
    bool bUsePreloadBinDepth_;
    bool bFlipImage_;
    int nMultiGrab_;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iNearD, iFarD;
	float binDepth_;
    std::string binName_;
    cv::Rect roiRect_;
    cv::VideoCapture kinect_;
    cv::Mat allBinSize_;
    std::vector<float> binSize_;
    Eigen::Vector3f kinectAngle_;
    float camera_backwall_distance_;
    BinBlobsDetector blobDetector_;

    bool loadCalibResults(std::string filename);
    bool loadParameters(std::string filename);
    bool copyParamFile(std::string src, std::string des);
    bool getBlobsByPointCluster(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs);
    bool getBlobsByColor(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs);
    bool getBlobsByRGBD(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs);
    bool filterBlobs(cv::Mat& blobs, int iBlobs);
    int countBlobs(cv::Mat& blobs);
};


#endif