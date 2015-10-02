#ifndef REGISTRATION_H
#define REGISTRATION_H


#pragma once

#ifdef _IS_LINUX_
#include <ros/ros.h>
#include "registration/SRV_Registration.h"
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <boost/filesystem.hpp>


#ifdef _IS_LINUX_
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#else
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#endif // _IS_LINUX_

#include <iostream>
#include <algorithm>

#define BOUND_BOX 100

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
#ifdef _IS_LINUX_
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
#else
typedef pcl::FPFHEstimation<PointNT,PointNT,FeatureT> FeatureEstimationT;
#endif // _IS_LINUX_
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//#ifdef _IS_WINDOWS_
#define DEBUG_MODE0
//#endif


class ImageMatching
{
public:
    ImageMatching(void);
    ~ImageMatching(void);
    
    bool extractFeatures(cv::Mat& img, std::vector<cv::KeyPoint>& kps, cv::Mat& des);
    float matchImage(cv::Mat& img1, cv::Mat& img2);
    float matchImage(cv::Mat& img1, cv::Mat& img2, cv::Mat& homography);
    std::vector<float> matchImage(cv::Mat& ob_img, std::vector<cv::Mat>& db_imgs, std::vector<std::vector<cv::KeyPoint> >& db_kps, std::vector<cv::Mat>& db_des, std::vector<cv::Mat>& homography);
    void setBinName(std::string bin_name) { bin_name_ = bin_name; }

private:
    std::string bin_name_;

    float findHomography( const std::vector<cv::KeyPoint>& source, const std::vector<cv::KeyPoint>& result, const std::vector<cv::DMatch>& input, std::vector<cv::DMatch>& inliers, cv::Mat& homography);
    void bfMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches, bool filterMatches = true );
    void flannMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches, bool filterMatches = true );
    void knnMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches );
    bool goodHomography(cv::Mat& homography);
};


class PoseEstimator
{
public:
    PoseEstimator(void);
    ~PoseEstimator(void);
    
    struct EstimatedResult
    {
        Eigen::Matrix4f transformation;
        float confidence;

        EstimatedResult(): confidence(0), transformation(Eigen::Matrix4f::Identity()) {};

        bool operator < (const EstimatedResult& rhs) const { return confidence < rhs.confidence; } // Ascending Sort
        bool operator > (const EstimatedResult& rhs) const { return confidence > rhs.confidence; } // Descending Sort
    };

    typedef enum 
    {
        FEATURE,
        ICP,
        FEA_ICP,
        PLANE,
        BOX,
        ROT_BOX,
        ANY
    } MethodType;

    float getPose(
        PointCloudT::Ptr object, 
        PointCloudT::Ptr scene, 
        PointCloudT::Ptr object_aligned, 
        Eigen::Matrix4f& transformation);

    float getPose(
        std::string cloud_file,
        std::string mask_file,
        std::string mesh_file,
        int blob_index,
        std::vector<float>& rotations,
        Eigen::Matrix4f& transformation);

    float getPose(
        std::string cloud_file,
        std::string mask_file,
        std::string mesh_file,
        int blob_index,
        std::vector<float>& rotations, 
        std::vector<float>& pose);

    void showAlignment(
        PointCloudT::Ptr object, 
        PointCloudT::Ptr scene, 
        PointCloudT::Ptr object_aligned, 
        Eigen::Matrix4f& transformation);

    void transformationToPose(Eigen::Matrix4f& transformation, std::vector<float>& pose);
    void extractBlob(cv::Mat& mask, int blob_index, cv::Mat& blob);
    void copyCloud(cv::Mat& cvCloud, PointCloudT::Ptr pclCloud);
    bool loadModel(std::string filename, PointCloudT::Ptr model);
    void setBinName(std::string binname) { bin_name_ = binname; printf("\nCurrent bin for registration: %s\n", bin_name_.c_str()); }
    void setBlobImage(cv::Mat color) { imBlob_ = color.clone(); }
    std::string getMethodTypeString(int type);

#ifdef _IS_LINUX_
    PoseEstimator(ros::NodeHandle& nh);
    ros::ServiceServer service_;
    bool getPose(registration::SRV_Registration::Request &req, registration::SRV_Registration::Response &res);
#endif

private:
    struct MatchingParams
    {
        int     METHOD_TYPE;
        float   VOXEL_Unit;
        double  FEAT_NormalEstimateRadius;
        double  FEAT_FeatureEstimateRadius;
        int     FEAT_AlignSampleNumber;
        int     FEAT_AlignCorrespondenceRandomness;
        float   FEAT_AlignSimilarityThreshold;
        float   FEAT_AlignMaxCorrespondenceDistanceFactor;
        float   FEAT_AlignInlierFraction;

        MatchingParams() :
            METHOD_TYPE (0),
            VOXEL_Unit (0.005),
            FEAT_NormalEstimateRadius (0.01),
            FEAT_FeatureEstimateRadius (0.025),
            FEAT_AlignSampleNumber (3),
            FEAT_AlignCorrespondenceRandomness (2),
            FEAT_AlignSimilarityThreshold (0.9f),
            FEAT_AlignMaxCorrespondenceDistanceFactor (1.5f),
            FEAT_AlignInlierFraction (0.25f)
        {};
    };

    float confidence_;
    std::string model_name_;
    std::string bin_name_;
    cv::Vec3f model_size_;
    cv::Vec3f object_size_;
    cv::Mat plane_map_;
    cv::Mat imBlob_;
    PointCloudT cloud_;
    MatchingParams param_;
    std::vector<Eigen::Matrix4f> initial_candidate_poses_;

#ifdef DEBUG_MODE
    pcl::visualization::PCLVisualizer viewer_;
#endif // DEBUG_MODE


    void loadParameters(std::string filename);

    float icpMatching(
        PointCloudT::Ptr object, 
        PointCloudT::Ptr scene, 
        PointCloudT::Ptr object_aligned, 
        Eigen::Matrix4f& transformation);

    float featureMatching(
        PointCloudT::Ptr object, 
        PointCloudT::Ptr scene, 
        PointCloudT::Ptr object_aligned, 
        Eigen::Matrix4f& transformation);

    void getBoundingBox(
        PointCloudT::Ptr cloud, 
        Eigen::Quaternionf& q, 
        Eigen::Vector3f& t,
        PointNT& max_point,
        PointNT& min_point,
        bool rotated = true);

    void adjustOrientation(Eigen::Affine3f& aff_src, Eigen::Affine3f& aff_dst);

    float getPlanePose(PointCloudT::Ptr scene, PointCloudT::Ptr model, Eigen::Matrix4f& transformation);

    float getProjectArea(PointCloudT::Ptr cloud, pcl::ModelCoefficients::Ptr plane_coeff);

    void matchModelFaceToObjectPlane(PointCloudT::Ptr model, PointCloudT::Ptr plane, Eigen::Vector3f orie_plane, Eigen::Matrix4f& transformation);

    int matchModelFaceWithImageFeature();

    void computePlaneDimension(PointCloudT::Ptr plane, Eigen::Vector3f orie_plane, float& plane_area, Eigen::Vector3f& plane_size, float& plane_angle);

    float getRotatedBoundingBoxPose(PointCloudT::Ptr scene, Eigen::Matrix4f& transformation);
    
    float getBoundingBoxPose(PointCloudT::Ptr scene, Eigen::Matrix4f& transformation);

    void printTransformation(Eigen::Matrix4f transformation);

    void removeSmallCluster(PointCloudT::Ptr srcCloud, PointCloudT::Ptr dstCloud);
};

#endif
