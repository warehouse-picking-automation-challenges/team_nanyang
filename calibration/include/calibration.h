#ifndef Calibration_H
#define Calibration_H


#pragma once

#ifdef _IS_LINUX_
#include <ros/ros.h>
#endif

#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
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

#define DEFAULT_I_LOW_H 0
#define DEFAULT_I_HIGH_H 179
#define DEFAULT_I_LOW_S 0
#define DEFAULT_I_HIGH_S 255
#define DEFAULT_I_LOW_V 170
#define DEFAULT_I_HIGH_V 255
#define DEFAULT_I_NEAR_D 400
#define DEFAULT_I_FAR_D 800
#define DEFAULT_BIN_DEPTH_MM 400

class MarkerBasedCalibration
{
public:
    MarkerBasedCalibration();
    ~MarkerBasedCalibration();

    bool processFrame(cv::Mat& rgb, cv::Mat& depth, cv::Mat& cloud, bool newFrame = true);

protected:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef pcl::PointXYZRGB ColorPointT;
    typedef pcl::PointCloud<ColorPointT> ColorCloudT;

    float bin_depth_;
    float MARKER_RADIUS_;
    int MARKER_COUNT_;
    int SIZE_VERT_WALL_, SIZE_HORI_WALL_;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iNearD, iFarD;

    cv::Mat allBinSize_;
    cv::Mat color_;
    cv::Mat depth_;
    cv::Mat cloud_;
    CloudT grid_points_;
    CloudT bar_centers_;
    CloudT camera_model_;
    ColorCloudT::Ptr pcl_cloud_;
    Eigen::Vector4f back_wall_coeff_;
    Eigen::Vector4f front_bar_coeff_;
    Eigen::Affine3f kinect_orient_;
    cv::Vec3f kinect_angle_;
    bool found_back_wall_, found_front_bar_, found_markers_, new_frame_;
    std::vector<cv::Point3f> marker_coordinates_;
    //cv::Mat markers_on_shelf_;
  
    pcl::visualization::PCLVisualizer viewer_;

    void initWindow();
    void hsvSpectrum();
    void copyCloud(cv::Mat& color, cv::Mat& cvCloud, ColorCloudT::Ptr pclCloud);
    bool estimateShelfDimensions();
    void showResults();
    bool validateMarkers(cv::Mat& depth, cv::Point center, int radius);
    bool loadParameters(std::string filename);
    bool saveParameters(std::string filename);
    bool saveResults(std::string filename);
    bool copyParamFile(std::string src, std::string des);
    bool getMarkers(const cv::Mat& color, const cv::Mat& depth, int iMarkers);
    bool filterMarkers(cv::Mat& blobs, int iBlobs);
};

#endif
