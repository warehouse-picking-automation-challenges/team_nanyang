#include "registration.h"

//////////////////////////////////////////////////////////////////////////
// fix the bug that Eigen's dot product of normalized vector exceed 1 or -1
float vectorAngle(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    float dp = (v1.normalized()).dot(v2.normalized());
    if (dp>1) dp=1;
    else if (dp<-1) dp=-1;
    return std::acos(dp);
}

// get system time string
std::string getCurrTimeString()
{
    time_t rawtime;
    struct tm *ts;
    char buf[80];

    time(&rawtime);
    ts = localtime(&rawtime);
    strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", ts);

    return buf;
}

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void listFiles(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) 
            ret.push_back(it->path().filename());
        ++it;
    }
}



//////////////////////////////////////////////////////////////////////////
PoseEstimator::PoseEstimator()
    : confidence_ (0)
{
    loadParameters(_PARAM_DIRECTORY_"/param_matching.yml");
}

PoseEstimator::~PoseEstimator()
{
#ifdef DEBUG_MODE
    viewer_.close();
#endif // DEBUG_MODE
    std::cout << "\nRegistration node STOPPED!\n\n";
}

#ifdef _IS_LINUX_

PoseEstimator::PoseEstimator(ros::NodeHandle& nh)
    : confidence_ (0)
{
    loadParameters(_PARAM_DIRECTORY_"/param_matching.yml");

    service_ = nh.advertiseService("get_pose", &PoseEstimator::getPose, this);
    ROS_INFO("Registration Service started.");
}

bool PoseEstimator::getPose(registration::SRV_Registration::Request &req, registration::SRV_Registration::Response &res)
{
    try 
    {
        int blob_index = req.blob_index;
        std::string cloud_file = req.capture_folder + "/point_cloud.yml";
        std::string mask_file = req.capture_folder + "/maskImage.png";
        std::string color_file = req.capture_folder + "/rgbImage.png";
        std::vector<float> rotations;
        std::vector<float> pose;

        int idx = mask_file.find("bin_");
        if (idx >= 0) 
        {
            bin_name_ = mask_file.substr(idx, 5);
            setBinName(bin_name_);
        }
        
        for (int i = 0; i < req.rotations.size(); i++)
            rotations.push_back( req.rotations[i] );

        imBlob_ = cv::imread(color_file);
        res.confidence = getPose(cloud_file, mask_file, req.object_name, blob_index, rotations, pose);

        for (int j = 0; j < pose.size(); j++) 
            res.pose.push_back( pose[j] );
    }
    catch (...) 
    {
        res.confidence = 0;
    }
}


#endif // _IS_LINUX_


void PoseEstimator::extractBlob(cv::Mat& mask, int blob_index, cv::Mat& blob)
{
    try
    {
        std::vector<uchar> uniqueLevels;
        cv::MatConstIterator_<uchar> pMask = mask.begin<uchar>(), pMask_end = mask.end<uchar>();

        for (; pMask != pMask_end; pMask++) 
        {
            bool isUnique = true;
            for (int i = 0; i < uniqueLevels.size(); i++)
            {
                if (*pMask == uniqueLevels[i])
                {
                    isUnique = false;
                    break;
                }
            }

            if (isUnique) 
            {
                uniqueLevels.push_back(*pMask); // includes '0' value
            }
        }

        if (uniqueLevels.size() > 1) 
        {
            std::sort(uniqueLevels.begin(), uniqueLevels.end()); // ascending order
            uniqueLevels.erase(uniqueLevels.begin());

            blob_index %= uniqueLevels.size();
            int desiredLevel = uniqueLevels[ blob_index];

            blob = cv::Mat::zeros(mask.size(), CV_8UC1);
            pMask = mask.begin<uchar>();
            cv::MatIterator_<uchar> pBlob = blob.begin<uchar>();

            for (; pMask != pMask_end; pMask++, pBlob++) 
            {
                if (*pMask == desiredLevel)
                    *pBlob = 255;
            }

        }
    }
    catch (...)
    {
        blob = cv::Mat();
    }
}

void PoseEstimator::copyCloud(cv::Mat& cvCloud, PointCloudT::Ptr pclCloud)
{
    pclCloud->clear();

    for(int r=0; r<cvCloud.rows; r++)
    {
        cv::Point3f* cloud_ptr = (cv::Point3f*)cvCloud.ptr(r);
        for(int c=0; c<cvCloud.cols; c++)
        {
            if (cloud_ptr[c].z > 0)
            {
                PointNT pt;
                pt.x = cloud_ptr[c].x;
                pt.y = cloud_ptr[c].y;
                pt.z = -cloud_ptr[c].z;

                pclCloud->points.push_back(pt);
            } 
        }
    }
}

void PoseEstimator::transformationToPose(Eigen::Matrix4f& transformation, std::vector<float>& pose)
{
    pose.resize(9);

    // Translation
    pose[0] = transformation.block<3,1>(0,3)[0]; 
    pose[1] = transformation.block<3,1>(0,3)[1];
    pose[2] = transformation.block<3,1>(0,3)[2];   
    if (pose[2] < 0) pose[2] = -pose[2];

    // Euler Angle
    Eigen::Affine3f aff_src(transformation), aff_dst;
    if (param_.METHOD_TYPE != BOX && param_.METHOD_TYPE != ROT_BOX) 
        adjustOrientation(aff_src, aff_dst);
    else
        aff_dst = aff_src;
    pcl::getEulerAngles(aff_dst, pose[3], pose[4], pose[5]);

    // Size
    pose[6] = object_size_[0];
    pose[7] = object_size_[1];
    pose[8] = object_size_[2];

    printf("\nObject orientation: %f, %f, %f\n\n", RAD2DEG(pose[3]), RAD2DEG(pose[4]), RAD2DEG(pose[5]));
}

bool PoseEstimator::loadModel(std::string model_name, PointCloudT::Ptr model)
{
    bool result = false;

    std::string filename = _LIB3D_DIRECTORY_"/" + model_name + "/standard.obj";

    // load model point cloud
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(filename, mesh);
    if (mesh.cloud.data.size() != 0) 
    {
        pcl::fromPCLPointCloud2(mesh.cloud, *model);

        model_name_ = model_name;

        cv::FileStorage fs;
        if (fs.open(_LIB3D_DIRECTORY_"/model_size.yml", cv::FileStorage::READ)) 
        {
            fs[model_name_] >> model_size_;
        }

        if (fs.open(_LIB3D_DIRECTORY_"/model_unit_is_mm.yml", cv::FileStorage::READ)) 
        {
            int unit_mm;
            fs[model_name_] >> unit_mm;

            if (unit_mm) 
            {
                printf("Scaling model points' unit from Millimeter to Meter ...\n");

                Eigen::Vector4f cenPt;
                pcl::compute3DCentroid(*model, cenPt);
                for (int i = 0; i < model->points.size(); ++i) 
                {
                    model->points[i].x -= cenPt[0];
                    model->points[i].y -= cenPt[1];
                    model->points[i].z -= cenPt[2];

                    model->points[i].x /= 1000;
                    model->points[i].y /= 1000;
                    model->points[i].z /= 1000;
                }
                
            }
        }

        if (fs.open(_LIB3D_DIRECTORY_"/model_match_method.yml", cv::FileStorage::READ)) 
        {
            fs[model_name_] >> param_.METHOD_TYPE;
        }

        std::cout << "\nModel Name: " << model_name_ << std::endl;
        std::cout << "Model Size: " << model_size_ << std::endl;
        std::cout << "Matching Method: " << getMethodTypeString(param_.METHOD_TYPE) << std::endl << std::endl;

        result = true;
    }

    return result;
}

std::string PoseEstimator::getMethodTypeString(int type)
{
    switch (type)
    {
    case FEATURE:
        return "FEATURE";
    case ICP:
        return "ICP";
    case FEA_ICP:
        return "FEA_ICP";
    case PLANE:
        return "PLANE";
    case BOX:
        return "BOX";
    default:
        return "UNKNOWN";
    }
}

float PoseEstimator::getPose(std::string cloud_file, 
                             std::string mask_file, 
                             std::string mesh_file, 
                             int blob_index, 
                             std::vector<float>& rotations, 
                             std::vector<float>& pose)
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float confidence = 0;

    confidence = getPose(cloud_file, mask_file, mesh_file, blob_index, rotations, transformation);
    transformationToPose(transformation, pose);

    return confidence;
}

float PoseEstimator::getPose(std::string cloud_file, 
                             std::string mask_file, 
                             std::string mesh_file, 
                             int blob_index, 
                             std::vector<float>& rotations,
                             Eigen::Matrix4f& transformation)
{
    cv::Mat mask, blob, point_cloud, object_cloud;
    PointCloudT::Ptr pSceneCloud (new PointCloudT), pModelCloud (new PointCloudT), pModelCloudAligned (new PointCloudT);
    std::vector<EstimatedResult> temp_result;
    float HIGH_CONFINDENCE_THRESH = 0.8;
    float confidence = 0;

    // load object point cloud
    cv::FileStorage fs(std::string(cloud_file).c_str(), cv::FileStorage::READ);
    if (fs.isOpened()) 
    {
        fs["Point Cloud"] >> point_cloud;
    }

    // load mask image
    mask = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);

    // load model
    loadModel(mesh_file, pModelCloud);

    // pose estimation
    if (!point_cloud.empty() && !mask.empty() && !pModelCloud->empty()) 
    {
        // extract recognized blob from mask image
        extractBlob(mask, blob_index, blob);

        if (!blob.empty()) 
        {
            // get object cloud
            point_cloud.copyTo(object_cloud, blob);
            copyCloud(object_cloud, pSceneCloud);
            
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(blob.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            cv::Rect roiRect = cv::boundingRect(contours[0]);
            cv::Mat imRoi = imBlob_(roiRect).clone();
            imBlob_ = imRoi.clone();

            //Eigen::Vector4f centroid;
            //pcl::compute3DCentroid(*pSceneCloud, centroid);

//            // find best match from possible viewpoints
            int N = rotations.size() / 9;
            std::cout << N << " possible viewpoints\n";

            initial_candidate_poses_.clear();
            int p = 0;
            for (int kk = 0; kk < N; kk++)
            {
                Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
                for (int ii = 0; ii < 3; ii++)
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        tmp_transformation(ii,jj) = rotations[p++];
                    }
                    //tmp_transformation(ii,3) = centroid[ii];
                }
                if (tmp_transformation != Eigen::Matrix4f::Identity())
                    initial_candidate_poses_.push_back(tmp_transformation);
            }

//                pcl::transformPointCloud(*pModelCloud, *pModelCloud, tmp_transformation);

                EstimatedResult result;
                result.confidence = getPose(pModelCloud, pSceneCloud, pModelCloudAligned, result.transformation);
                
                temp_result.push_back(result);

//                if (result.confidence > HIGH_CONFINDENCE_THRESH)
//                    break;
//            }

//            std::sort(temp_result.begin(), temp_result.end(), std::greater<EstimatedResult>());

            confidence = temp_result[0].confidence;
            transformation = temp_result[0].transformation;
        }
    }

    return confidence;
}

float PoseEstimator::getPose(PointCloudT::Ptr model_src, 
                             PointCloudT::Ptr object_src, 
                             PointCloudT::Ptr model_aligned, 
                             Eigen::Matrix4f& transformation)
{
    std::cout << "\n=====>> Pose Estimation Started <<=====\n";

    pcl::copyPointCloud(*object_src, cloud_);

    // Downsample
    PointCloudT::Ptr model (new PointCloudT); 
    PointCloudT::Ptr object (new PointCloudT);
    const float leaf = param_.VOXEL_Unit;

    pcl::VoxelGrid<PointNT> vg;
    vg.setLeafSize (leaf, leaf, leaf);
    vg.setInputCloud (model_src);
    vg.filter (*model);
    vg.setInputCloud (object_src);
    vg.filter (*object);

    removeSmallCluster(object, object);
    removeSmallCluster(model, model);

    pcl::copyPointCloud(*object, *object_src);
    pcl::copyPointCloud(*model, *model_src);

    MethodType type = static_cast<MethodType>(param_.METHOD_TYPE);

    // Alignment
    if (type == MethodType::FEATURE)
    {
        confidence_ = featureMatching(model, object, model_aligned, transformation);
    }
    else if (type == MethodType::ICP)
    {
        confidence_ = icpMatching(model, object, model_aligned, transformation);
    }
    else if (type == MethodType::FEA_ICP)
    {
        Eigen::Matrix4f transMat_1 = Eigen::Matrix4f::Identity(), transMat_2 = Eigen::Matrix4f::Identity();
        PointCloudT::Ptr object_aligned_1 (new PointCloudT);
        float confidence_1, confidence_2;

        confidence_1 = featureMatching(model, object, object_aligned_1, transMat_1);
        confidence_2 = icpMatching(object_aligned_1, object, model_aligned, transMat_2);

        confidence_ = confidence_1 > 0 ? confidence_1 * confidence_2 : confidence_2;
        transformation = confidence_1 > 0 ? transMat_2 * transMat_1 : transMat_2;
    }
    else if (type == MethodType::PLANE) 
    {
        confidence_ = getPlanePose(object, model, transformation);
    }
    else if (type == MethodType::BOX)
    {
        confidence_ = getBoundingBoxPose(object, transformation);
    }
    else if (type == MethodType::ROT_BOX)
    {
        confidence_ = getRotatedBoundingBoxPose(object, transformation);
    }
    else if (type = MethodType::ANY) 
    {
        confidence_ = featureMatching(model, object, model_aligned, transformation);
        if (confidence_ == 0) 
        {
            //confidence_ = icpMatching(object, object, object_aligned, transformation);
            if (confidence_ == 0) 
            {
                confidence_ = getPlanePose(object, model, transformation);
                if (confidence_ == 0) 
                {
                    confidence_ = getBoundingBoxPose(object, transformation);
                }
            }
        }
    }

    showAlignment(model_src, object_src, model_aligned, transformation);

    return confidence_;
}

float PoseEstimator::getPlanePose(PointCloudT::Ptr scene0, 
                                  PointCloudT::Ptr model,
                                  Eigen::Matrix4f& transformation)
{
    std::cout << "\nPlane matching Started.\n";
    float confidence = 1;

    object_size_ = model_size_;

    // Detect all planar surface
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ()),
        scene_p (new pcl::PointCloud<pcl::PointXYZ> ()),
        facing_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    PointCloudT::Ptr plane (new PointCloudT ());
    std::vector<pcl::ModelCoefficients> plane_coeffs;

    Eigen::Vector3f orie_ref(0,0,1); 
    Eigen::Vector3f orie_plane;
    Eigen::Vector3f rot_axis, plane_angle;
    Eigen::Affine3f t;
    Eigen::Quaternionf q;
    float theta, min_theta = 9e9;
    int min_id = -1;

    pcl::copyPointCloud(*scene0, *scene);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    int nr_points = (int) scene->points.size ();

    while (scene->points.size () > 0.1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setInputCloud (scene);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        orie_plane[0] = coefficients->values[0];
        orie_plane[1] = coefficients->values[1];
        orie_plane[2] = coefficients->values[2];
        theta = std::acos( orie_plane.dot(orie_ref) );
        rot_axis = (orie_ref.cross(orie_plane)).normalized();
        q = Eigen::AngleAxisf( -theta, rot_axis );
        t = q.normalized();

        pcl::getEulerAngles(t, plane_angle[0], plane_angle[1], plane_angle[2]);

        plane_coeffs.push_back(*coefficients);

        if (min_theta > std::abs(theta)) 
        {
            min_theta = std::abs(theta);
            min_id = plane_coeffs.size() - 1;
        }

        printf("#%i plane coefficients: %f, %f, %f, %f\n", 
            plane_coeffs.size(),
            coefficients->values[0], 
            coefficients->values[1], 
            coefficients->values[2], 
            coefficients->values[3]);
        printf("   plane angle: %f, %f, %f\n   angle with Z axis: %f\n", 
            RAD2DEG(plane_angle[0]), RAD2DEG(plane_angle[1]), RAD2DEG(plane_angle[2]),
            RAD2DEG(theta));

        // Extract the planar inliers 
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (scene);
        extract.setIndices (inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*scene_p);
        *scene = *scene_p;
    }

    if (min_id < 0)
    {
        confidence  = 0;
        std::cout << "Plane matching FAILED!" << std::endl;
    }
    else
    {
        // Select the one best align with Z axis    
        Eigen::Vector4f facing_plane_coeff;
        for (size_t ii=0;ii<4;++ii)
            facing_plane_coeff[ii] = plane_coeffs[min_id].values[ii];

        pcl::copyPointCloud(*scene0, *scene);
        float thresh = 0.015f;
        std::vector<int> inliers; 
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr scmp (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (scene)); 
        scmp -> selectWithinDistance (facing_plane_coeff, thresh, inliers); 
        pcl::copyPointCloud(*scene0, inliers, *facing_plane); 

        // get plane centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*facing_plane, centroid);
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*facing_plane, minPt, maxPt);

        // estimate the plane belongs to which face of the model (front/left/top, box-like only) 
        // transform that face of model to align with Z axis
        Eigen::Vector3f orie_model(0,0,1); // assume orie_model = Z axis
        Eigen::Vector3f orie_plane(facing_plane_coeff[0], facing_plane_coeff[1], facing_plane_coeff[2]);
        
        Eigen::Matrix4f transToZaxis;
        pcl::copyPointCloud(*facing_plane, *plane);
        matchModelFaceToObjectPlane(model, plane, orie_plane, transToZaxis);

        // compute transformation
        theta = std::acos( orie_plane.dot(orie_model) );
        rot_axis = (orie_model.cross(orie_plane)).normalized();
        q = Eigen::AngleAxisf( theta, rot_axis );
        t = q.normalized();
        t.translation() << centroid[0], maxPt.y - object_size_[1]*0.5, centroid[2];

        transformation = t.matrix() * transToZaxis;

        std::cout << "Transformation of aligning model to object: \n";
        printTransformation(transformation);

#ifdef DEBUG_MODE
        float factor = 0.2;
        PointNT pt1, pt2;
        pt1.x = centroid[0]; pt1.y = centroid[1]; pt1.z = centroid[2];
        pt2.x = pt1.x + facing_plane_coeff[0] * factor;
        pt2.y = pt1.y + facing_plane_coeff[1] * factor;
        pt2.z = pt1.z + facing_plane_coeff[2] * factor;

        viewer_.removeAllPointClouds();
        viewer_.removeAllShapes();
#if PCL_VERSION_COMPARE(>,1,7,0)
        viewer_.removeAllCoordinateSystems();
#else
        viewer_.removeCoordinateSystem();
#endif

        viewer_.addCoordinateSystem(0.1);
        viewer_.addPointCloud(facing_plane);
        viewer_.addArrow(pt2, pt1, 1, 0.5, 0.5, false);

        PointNT pt11, pt22;
        pt11.x = pt11.y = pt11.z = 0;
        pt22.x = pt11.x + facing_plane_coeff[0] * factor;
        pt22.y = pt11.y + facing_plane_coeff[1] * factor;
        pt22.z = pt11.z + facing_plane_coeff[2] * factor;
        viewer_.addArrow(pt22, pt11, 1, 0.5, 0.5, false, "arr2");

        PointNT pt33;
        pt33.x = pt11.x + rot_axis[0] * factor;
        pt33.y = pt11.y + rot_axis[1] * factor;
        pt33.z = pt11.z + rot_axis[2] * factor;
        viewer_.addArrow(pt33, pt11, 0.5, 1, 0.5, false, "arr3");

        PointNT pt44;
        pt44.x = pt11.x + orie_model[0] * factor;
        pt44.y = pt11.y + orie_model[1] * factor;
        pt44.z = pt11.z + orie_model[2] * factor;
        viewer_.addArrow(pt44, pt11, 0.5, 0.5, 1, false, "arr4");

        viewer_.spin();
#endif // DEBUG_MODE

    }

    return confidence;
}

void PoseEstimator::computePlaneDimension(PointCloudT::Ptr plane, 
                                          Eigen::Vector3f orie_plane,
                                          float& plane_area, 
                                          Eigen::Vector3f& plane_size, 
                                          float& plane_angle)
{
    // rotate the plane to align to Z axis
    PointCloudT::Ptr plane_rot (new PointCloudT());
    Eigen::Vector3f rot_axis;
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q;
    Eigen::Vector3f orie_ref(0,0,1);

    float theta = std::acos( orie_plane.dot(orie_ref) );
    rot_axis = (orie_ref.cross(orie_plane)).normalized();
    q = Eigen::AngleAxisf( -theta, rot_axis );
    t = q.normalized();

    pcl::transformPointCloud(*plane, *plane_rot, t.matrix());

    std::cout << "\nTransformation of aligning object plane to Z axis:\n";
    printTransformation(t.matrix());

    // map plane cloud onto a cv::Mat
    PointNT maxPt, minPt;
    pcl::getMinMax3D(*plane, minPt, maxPt);
    int margin = 5;
    float unit = param_.VOXEL_Unit;
    int width = (int)((maxPt.x - minPt.x) / unit) + margin*2;
    int height = (int)((maxPt.y - minPt.y) / unit) + margin*2;

    plane_map_ = cv::Mat::zeros(height, width, CV_8UC1);

    for (size_t i=0; i<plane->points.size(); ++i) 
    {
        int x = (int)((plane->points[i].x - minPt.x) / unit) + margin;
        int y = (int)((plane->points[i].y - minPt.y) / unit) + margin;
        y = height - y;
        plane_map_.at<uchar>(y,x) = 255;
    }

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(plane_map_, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    float max_area = -1;
    int max_id = 0;
    for (size_t j=0; j<contours.size(); ++j) 
    {
        float area = cv::contourArea(contours[j]);
        if ( area > max_area) 
        {
            max_area = area;
            max_id = j;
        }
    }

    cv::drawContours(plane_map_, contours, max_id, cv::Scalar::all(255), -1);

    cv::RotatedRect rot_rect = cv::minAreaRect( cv::Mat(contours[max_id]) );
    float blob_angle_deg = rot_rect.angle;
    //if (rot_rect.size.width < rot_rect.size.height) 
    //{
    //    blob_angle_deg = 90 + blob_angle_deg;
    //}
    plane_angle = blob_angle_deg;
    plane_size[0] = rot_rect.size.width * unit;
    plane_size[1] = rot_rect.size.height * unit;
    plane_size[2] = 0;
    plane_area = plane_size[0] * plane_size[1];

//#ifdef DEBUG_MODE
//    imshow("object plane", plane_map_);
//    cv::waitKey();
//#endif // DEBUG_MODE
}

int PoseEstimator::matchModelFaceWithImageFeature()
{
    int faceIndex = -1;

    try {
        std::string filename = _LIB3D_DIRECTORY_"/" + model_name_ + "/feature.yml";
        cv::FileStorage fs(filename, cv::FileStorage::READ);

        if (fs.isOpened()) 
        {
            std::vector<std::vector<cv::KeyPoint> > obj_kps_set(6);
            std::vector<cv::Mat> obj_des_set(6), obj_img_set(6);
            std::string faces[6] = {"front", "right", "back", "left", "up", "down"};
            cv::FileNode node;
            for (int i=0;i<6;i++) 
            {
                node = fs[ faces[i]+"_keypoint" ]; cv::read(node, obj_kps_set[i]);
                node = fs[ faces[i]+"_descriptor" ]; cv::read(node, obj_des_set[i]);
                node = fs[ faces[i]+"_img" ]; cv::read(node, obj_img_set[i]);
            }

            std::vector<float> match_score;
            std::vector<cv::Mat> match_homography;
            ImageMatching matcher;

            matcher.setBinName( bin_name_ );
            match_score = matcher.matchImage(imBlob_, obj_img_set, obj_kps_set, obj_des_set, match_homography);

            std::vector<std::pair<float, size_t> > vp;
            vp.reserve(match_score.size());
            for (size_t i = 0 ; i != match_score.size() ; i++) {
                vp.push_back(std::make_pair(match_score[i], i));
            }

            // Sorting will put lower values ahead of larger ones,
            // resolving ties using the original index
            std::sort(vp.begin(), vp.end());

            float max_score = vp[vp.size()-1].first;
            if (max_score > 4) 
            {
                switch (vp[vp.size()-1].second)
                {
                case 0: 
                case 2:
                    faceIndex = 1; // front or back
                    break;
                case 1:
                case 3:
                    faceIndex = 3; // left or right
                    break;
                case 4:
                case 5:
                    faceIndex = 2; // top or bottom
                default:
                    faceIndex = -1;
                    break;
                } ;
            }
        }
    }
    catch (cv::Exception e) {
    	faceIndex = -1;
    }
    catch (...) {
        faceIndex = -1;
    }

    return faceIndex;
}

void PoseEstimator::matchModelFaceToObjectPlane(PointCloudT::Ptr model, 
                                                PointCloudT::Ptr plane,  
                                                Eigen::Vector3f orie_plane,
                                                Eigen::Matrix4f& transformation)
{
    if (object_size_[0] > 0) 
    {
        // compute plane size
        Eigen::Vector3f rot_axis;
        Eigen::Vector3f plane_size, plane_rect_size;
        float plane_area, plane_angle;
        float theta;
        computePlaneDimension(plane, orie_plane, plane_area, plane_size, plane_angle);      

        printf("Plane size: %.3f, %.3f\nModel size: %.3f, %.3f, %.3f\n",
            plane_size[0], plane_size[1], model_size_[0], model_size_[1], model_size_[2]);

        // determine corresponding face 
        int face_code = 0; //matchModelFaceWithImageFeature();
        if (face_code <= 0)
        {
            //printf("\nFace image matching FAILED! Use shape matching.\n");
            for (int i=0;i<2;++i)
            {
                float min_vs = 9e9;
                int min_id = -1;
                for (int j=0;j<3;++j)
                {
                    float vs = std::abs(model_size_[j]/plane_size[i] - 1);
                    if (min_vs > vs)
                    {
                        min_vs = vs;
                        min_id = j;
                    }
                }
                face_code += min_id;
            }
//            float model_areas[3] = { model_size_[0]*model_size_[1], model_size_[0]*model_size_[2], model_size_[1]*model_size_[2] };
//            float min_rate = 9e9;
//            for (int i=0;i<3;i++)
//            {
//                float rate = std::abs(plane_area/model_areas[i] - 1);
//                printf("Plane area: %f, Model face area: %f, Rate: %f\n", plane_area, model_areas[i], rate);
//                if (min_rate > rate)
//                {
//                    min_rate = rate;
//                    face_code = i+1;
//                }
//            }
        }
        else
            printf("\nFace image matching SUCCEEDED! \n");

        if (face_code == 3)
        {
            rot_axis = Eigen::Vector3f::UnitY();

            // rotate around Y axis, exchange the value of Width and Depth of the model
            float tmp_val = object_size_[0];
            object_size_[0] = object_size_[2];
            object_size_[2] = tmp_val;

            printf("The object plane is the LEFT or RIGHT face of the model\n");
        } 
        //else if (area_rate.z < area_rate.x && area_rate.z < area_rate.y)
        else if (face_code == 2)
        {
            rot_axis = Eigen::Vector3f::UnitX();

            // rotate around X axis, exchange the value of Height and Depth of the model
            float tmp_val = object_size_[1];
            object_size_[1] = object_size_[2];
            object_size_[2] = tmp_val;

            printf("The object plane is the TOP or BOTTOM face of the model\n");
        }
        else
        {
            rot_axis = Eigen::Vector3f::UnitZ();

            printf("The object plane is the FRONT or BACK face of the model\n");
        }
        
        // compute transformation of rotating corresponding face to face the camera
        Eigen::Affine3f t1 = Eigen::Affine3f::Identity();
        theta = M_PI*0.5f;
        if (rot_axis != Eigen::Vector3f::UnitZ()) 
        {
            t1.rotate(Eigen::AngleAxisf(theta, rot_axis));

            if (rot_axis == Eigen::Vector3f::UnitX()) 
            {
                printf("Rotating model around X axis by 90 degree...\n");
            } 
            else if (rot_axis == Eigen::Vector3f::UnitY()) 
            {
                printf("Rotating model around Y axis by 90 degree...\n");
            }
        }

        // compute the angle of model face
        float unit = param_.VOXEL_Unit;
        std::vector<cv::Point2f> model_points(4);
        model_points[0] = cv::Point2f(0,0);
        model_points[1] = cv::Point2f(object_size_[0] / unit, 0);
        model_points[2] = cv::Point2f(object_size_[0] / unit, object_size_[1] / unit);
        model_points[3] = cv::Point2f(0, object_size_[1] / unit);

        cv::RotatedRect model_rect = cv::minAreaRect( cv::Mat(model_points) );
        float model_angle = model_rect.angle;

        printf("Dimension of object plane:\n Size: %f, %f\n Angle: %f\n",
            plane_size[0], plane_size[1], plane_angle);
        printf("Dimension of model face:\n Size: %f, %f\n Angle: %f\n",
            model_rect.size.width * unit, model_rect.size.height * unit, model_angle);

        // adjust model angle when the longer side of rotated rectangle         
        // of object and model are different
        Eigen::Affine3f t2 = Eigen::Affine3f::Identity();
        float w2h_plane = plane_size[0]/plane_size[1] - 1;
        float w2h_model = model_rect.size.width/model_rect.size.height - 1;

        if ( (w2h_plane * w2h_model) < 0 )
        {
            model_angle += 90;
        }

        // compute transformation of rotating corresponding face 
        // to the same roll angle as that of object plane
        theta = -(plane_angle - model_angle);
        if (135 > std::abs(theta) && std::abs(theta) > 45) 
        {
            float tmp_val = object_size_[0];
            object_size_[0] = object_size_[1];
            object_size_[1] = tmp_val;
        }

        t2.rotate(Eigen::AngleAxisf(DEG2RAD(theta), Eigen::Vector3f::UnitZ()));
        printf("Rotating model around Z axis by %f degree...\n", theta);

        t2.translation() << 0, 0, -object_size_[2]*0.5;

        // compute final transformation
        transformation = t2.matrix() * t1.matrix();        

        std::cout << "Transformation of matching model face to object plane: \n";
        printTransformation(transformation);
    }
}

float PoseEstimator::getBoundingBoxPose(PointCloudT::Ptr object, Eigen::Matrix4f& transformation)
{
    std::cout << "\nBounding box matching Started.\n";
    float confidence = 1;

    PointNT minPt, maxPt;
    Eigen::Quaternionf q;
    Eigen::Vector3f t;
    getBoundingBox(object, q, t, maxPt, minPt, false);

    object_size_[0] = std::abs(maxPt.x - minPt.x);
    object_size_[1] = std::abs(maxPt.y - minPt.y);
    object_size_[2] = std::abs(maxPt.z - minPt.z);

    Eigen::Affine3f af;
    af = q.normalized();
    af.translation() << t;

    transformation = af.matrix();

    return confidence;
}

float PoseEstimator::getRotatedBoundingBoxPose(PointCloudT::Ptr object, 
                                Eigen::Matrix4f& transformation)
{
    std::cout << "\nRotated bounding box matching Started.\n";
    float confidence = 1;

    // Compute bounding box
    Eigen::Quaternionf orie_box_origin, orie_box_origin_normlized;
    Eigen::Vector3f box_centroid;
    PointNT box_maxPt_origin, box_minPt_origin;
    getBoundingBox(object, orie_box_origin, box_centroid, box_maxPt_origin, box_minPt_origin);

#ifdef DEBUG_MODE
    viewer_.addCube(box_centroid, orie_box_origin, box_maxPt_origin.x - box_minPt_origin.x, box_maxPt_origin.y - box_minPt_origin.y, box_maxPt_origin.z - box_minPt_origin.z, "box");
    viewer_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "box");
    viewer_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "box");

    viewer_.addCoordinateSystem(0.2);
#endif // DEBUG_MODE


    // Transform maxPt and minPt to real position
    Eigen::Affine3f affine_box, affine_box_adjust;
    affine_box = orie_box_origin.normalized();
    affine_box.translation() << box_centroid;
    PointNT box_maxPt_real = pcl::transformPoint(box_maxPt_origin, affine_box);
    PointNT box_minPt_real = pcl::transformPoint(box_minPt_origin, affine_box);

    adjustOrientation(affine_box, affine_box_adjust);

    affine_box_adjust.translation() << box_centroid;
    
    // Final transformation
    transformation = affine_box_adjust.matrix();

    // Compute the size of bounding box
    object_size_[0] = std::abs(box_maxPt_real.x - box_minPt_real.x);
    object_size_[1] = std::abs(box_maxPt_real.y - box_minPt_real.y);
    object_size_[2] = std::abs(box_maxPt_real.z - box_minPt_real.z);

    return confidence;
}

void PoseEstimator::adjustOrientation(Eigen::Affine3f& aff_src, Eigen::Affine3f& aff_dst)
{
    std::cout << "Adjusting transformed coordinate to align with world coordinate\n";
    // Caculate which axis of the box is facing to the camera
    std::vector<Eigen::Vector3f> box_axis(3);
    Eigen::Vector3f ref_z(0,0,1), ref_y(0,1,0);
    int match_z = -1, match_y = -1;
    float theta, min_theta = 9e9, min_axis = -1;
    char xyz[] = "XYZ";

    for (int i=0;i<3;i++) 
    {
        Eigen::Vector3f orie_axis(0,0,0);
        orie_axis[i] = 1;
        box_axis[i] = (aff_src.rotation().matrix() * orie_axis).normalized();
        theta = vectorAngle( box_axis[i], ref_z );
        if (theta > M_PI_2)  theta -= M_PI;

        if (min_theta > std::abs(theta)) 
        {
            min_theta = std::abs(theta);
            match_z = i;
        }
#ifdef DEBUG_MODE
        printf("Angle between object %c axis and reference Z axis: %.3f\n", xyz[i], RAD2DEG(theta));
#endif
    }    

    // Compute first transformation
    Eigen::Vector3f rot_axis;
    Eigen::Affine3f align_z = Eigen::Affine3f::Identity();
    Eigen::Quaternionf quaternion_align_z;

    theta = vectorAngle( box_axis[match_z], ref_z );
    if (theta > M_PI_2)  theta -= M_PI;
    if (theta != 0) 
    {
        rot_axis = (ref_z.cross(box_axis[match_z])).normalized();
        quaternion_align_z = Eigen::AngleAxisf( theta, rot_axis );
        align_z = quaternion_align_z.normalized();
    }

    // Compute Y axis in align_z coordinate
    Eigen::Vector3f axis_y_new(0,1,0);
    axis_y_new = (quaternion_align_z.matrix() * ref_y).normalized();
    min_theta = 9e9; 
    min_axis = -1;

    for (int i=0;i<3;i++) 
    {
        if (i == match_z)
            continue;
       
        theta = vectorAngle(box_axis[i], axis_y_new);
        if (theta > M_PI_2)  theta -= M_PI;

        if (min_theta > std::abs(theta)) 
        {
            min_theta = std::abs(theta);
            match_y = i;
        }
#ifdef DEBUG_MODE
        printf("Angle between object %c axis and new reference Y axis: %.3f\n", 
            xyz[i], RAD2DEG(theta));
#endif // DEBUG_MODE
    }  

    Eigen::Affine3f align_y = Eigen::Affine3f::Identity();
    Eigen::Quaternionf quaternion_align_y;

    theta = vectorAngle(box_axis[match_y], axis_y_new);
    if (theta > M_PI_2)  theta -= M_PI;
    if (theta != 0) 
    {
        rot_axis = (axis_y_new.cross(box_axis[match_y])).normalized();
        quaternion_align_y = Eigen::AngleAxisf( theta, rot_axis );
        align_y = quaternion_align_y.normalized();
    }

    aff_dst = align_y * align_z;
    aff_dst.translation() << aff_src.translation();
}

float PoseEstimator::icpMatching(PointCloudT::Ptr object, 
                                 PointCloudT::Ptr scene, 
                                 PointCloudT::Ptr object_aligned, 
                                 Eigen::Matrix4f& transformation)
{
    std::cout << "ICP matching Started.\n";

    float confidence = 0;
    pcl::IterativeClosestPoint<PointNT, PointNT> icp;
    icp.setInputSource(object);
    icp.setInputTarget(scene);
    //icp.setMaxCorrespondenceDistance(0.01);
    //icp.setRANSACOutlierRejectionThreshold(0.01);
    //icp.setMaximumIterations(10000);
    //icp.setEuclideanFitnessEpsilon(0.1);
    //icp.setTransformationEpsilon(1e-8);

    icp.align(*object_aligned);

    if (icp.hasConverged()) 
    {
        transformation = icp.getFinalTransformation();
        confidence = 1 - icp.getFitnessScore();

        std::cout << "Results of ICP matching.\nConfidence: " << confidence << std::endl;
    }
    else
        std::cout << "ICP matching FAILED!\n";

    return confidence;
}


float PoseEstimator::featureMatching(PointCloudT::Ptr object, 
                                     PointCloudT::Ptr scene, 
                                     PointCloudT::Ptr object_aligned, 
                                     Eigen::Matrix4f& transformation)
{
    std::cout << "Feature matching Started.\n";
    float confidence = 0;

    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);
    pcl::search::KdTree<PointNT>::Ptr kdtree (new pcl::search::KdTree<PointNT>);
    const float leaf = param_.VOXEL_Unit;

    // Estimate normals for scene
#ifdef _IS_LINUX_
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
#else
    pcl::NormalEstimation<PointNT,PointNT> nest;
#endif
    nest.setRadiusSearch (param_.FEAT_NormalEstimateRadius);
    nest.setSearchMethod(kdtree);
    nest.setInputCloud (scene);
    nest.compute (*scene);
    nest.setInputCloud(object);
    nest.compute(*object);

    // Estimate features
    FeatureEstimationT fest;
    fest.setRadiusSearch (param_.FEAT_FeatureEstimateRadius);
    fest.setSearchMethod(kdtree);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    // Perform alignment
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setInputTarget (scene);
    align.setSourceFeatures (object_features);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (10000); // Number of RANSAC iterations
    align.setNumberOfSamples (param_.FEAT_AlignSampleNumber); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (param_.FEAT_AlignCorrespondenceRandomness); // Number of nearest features to use
    align.setSimilarityThreshold (param_.FEAT_AlignSimilarityThreshold); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (leaf * param_.FEAT_AlignMaxCorrespondenceDistanceFactor); // Inlier threshold
    align.setInlierFraction (param_.FEAT_AlignInlierFraction); // Required inlier fraction for accepting a pose hypothesis
    align.align (*object_aligned);

    if (align.hasConverged()) 
    {
        transformation = align.getFinalTransformation();
        confidence = 1 - align.getFitnessScore();

        std::cout << "Results of feature matching.\nConfidence: " << confidence << std::endl;
        std::cout << "Inliers: " << align.getInliers().size() << "/" << object->size() << std::endl;
    }
    else
        std::cout << "Feature matching FAILED!\n";

    return confidence;
}


void PoseEstimator::showAlignment(PointCloudT::Ptr model, 
                                  PointCloudT::Ptr object, 
                                  PointCloudT::Ptr model_aligned,  
                                  Eigen::Matrix4f& transformation)
{
    // Show results
    std::cout << "\nRegistration result\n";
    printTransformation(transformation);
    
#ifndef DEBUG_MODE
    static pcl::visualization::PCLVisualizer viewer_;
#endif

#ifdef _IS_WINDOWS_
    viewer_.removeAllCoordinateSystems();
#else
    viewer_.removeCoordinateSystem();
#endif 
    viewer_.removeAllPointClouds();
    viewer_.removeAllShapes();


	// Show alignment	
	viewer_.addPointCloud (object, ColorHandlerT (object, 0.0, 255.0, 0.0), "object");
	viewer_.addPointCloud (model, ColorHandlerT (model, 255.0, 0.0, 0.0), "model");
    if (param_.METHOD_TYPE != BOX && param_.METHOD_TYPE != ROT_BOX) 
    {
        // Compute transformed model
        pcl::transformPointCloud(*model, *model_aligned, transformation);
        if (model_aligned->points.size() > 0)
            viewer_.addPointCloud (model_aligned, ColorHandlerT (model_aligned, 0.0, 0.0, 255.0), "model_aligned");
    }

//    if (initial_candidate_poses_.size() > 0)
//    {
//        int N = initial_candidate_poses_.size();
//        float dist = MAX(model_size_[0], MAX(model_size_[1], model_size_[2])) * 1.5;
//        float theta = CV_PI * 2 / (N+1);
//        std::stringstream ss;
//        ss << "candidate_pose_";

//        for (size_t i=0; i<initial_candidate_poses_.size();i++)
//        {
//            Eigen::Affine3f af(initial_candidate_poses_[i]);
//            float dx = dist * cos(theta*i);
//            float dz = dist * sin(theta*i);
//            af.translation() << dx, 0, dz;
//            PointCloudT candi_cloud;
//            pcl::transformPointCloud(*model, candi_cloud, af);
//            ss << i;
//            viewer_.addPointCloud (candi_cloud.makeShared(), ColorHandlerT (model_aligned, 0.0, 250.0, 255.0), ss.str());
//        }
//    }

//    // Compute object bounding box
//    Eigen::Quaternionf q;
//    Eigen::Vector3f t;
//    PointNT maxPt, minPt;
//    getBoundingBox(object, q, t, maxPt, minPt, param_.METHOD_TYPE != BOX );

//    // Show bounding box
//    viewer_.addCube(t, q, maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z, "scene");
//    viewer_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "scene");
//    viewer_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "scene");

#ifdef DEBUG_MODE
    //float factor = 0.2;
    //PointNT pt1, pt2;
    //pt1.x = t[0]; pt1.y = t[1]; pt1.z = t[2];
    //pt2.x = pt1.x + a * factor;
    //pt2.y = pt1.y + b * factor;
    //pt2.z = pt1.z + c * factor;

    Eigen::Affine3f bf;
    bf = q.normalized();
    bf.translation() << t;

    //viewer_.addArrow(pt2, pt1, 0, 1, 0, false, "box_arr");
    viewer_.addCoordinateSystem(0.1, bf, BOUND_BOX);
#endif

        //viewer_.addCoordinateSystem(0.2);

//    Eigen::Affine3f af_s(transformation), af;
//    if (param_.METHOD_TYPE != BOX && param_.METHOD_TYPE != ROT_BOX)
//    {
//        adjustOrientation(af_s, af);
//    }
//    else
//    {
//        af = af_s;
//    }
//    #if PCL_VERSION_COMPARE(>,1,7,0)
//    viewer_.addCoordinateSystem(0.2, af, "transform");
//    #else
//    viewer_.addCoordinateSystem(0.1, af);
//    #endif

    viewer_.setCameraPosition(
        -0.689638, 0.353073, 0.305785, 
        0.0549899, -0.0643472, -0.21507, 
        0.436154, 0.894979, -0.0937106);

    std::string png_file = _OUTPUT_DIRECTORY_ "/../../capture/capture_kinect/output/images/" + bin_name_ + "/registration_result.png"; 
    //viewer_.saveScreenshot(png_file);

    viewer_.spinOnce();

    std::string date_str = getCurrTimeString().substr(0,8);
    std::string src_path = _OUTPUT_DIRECTORY_ "/../../capture/capture_kinect/output/images/" + bin_name_ + "/";
    std::string log_path = _OUTPUT_DIRECTORY_ "/../../capture/capture_kinect/output/images/trial_sets/" + date_str;
    std::string pre_name = bin_name_ + "-" + model_name_ + "-" + getCurrTimeString() + "-";
    std::string file_ext = ".png";
    boost::filesystem::path root(src_path);
    std::vector<boost::filesystem::path> img_files;
    boost::filesystem::create_directory(log_path);

    listFiles(root, file_ext, img_files);
    for (size_t i=0;i<img_files.size();i++) 
    {
        std::string src_file = src_path + img_files[i].filename().string();
        std::string dst_file = log_path + pre_name + img_files[i].filename().string();
        cv::imwrite(dst_file, cv::imread(src_file, -1));
    }

}

void PoseEstimator::printTransformation(Eigen::Matrix4f transformation)
{
    printf ("\n");
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    printf ("\n");
}

float PoseEstimator::getProjectArea(PointCloudT::Ptr cloud, pcl::ModelCoefficients::Ptr plane_coeff)
{
    float area;
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>()), 
        cloud_proj (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloudT::Ptr cloud_proj2 (new PointCloudT());

    pcl::copyPointCloud(*cloud, *cloud_xyz);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_xyz);
    proj.setModelCoefficients (plane_coeff);
    proj.filter (*cloud_proj);

    float angle;
    Eigen::Vector3f size;
    Eigen::Vector3f coeff(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);
    pcl::copyPointCloud(*cloud_proj, *cloud_proj2);
    computePlaneDimension(cloud_proj2, coeff, area, size, angle);

    area = size[0]*size[1];

    //// Create a Concave Hull representation of the projected inliers
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::ConcaveHull<pcl::PointXYZ> chull;
    //chull.setInputCloud (cloud_proj);
    //chull.setAlpha (0.1);
    //chull.reconstruct (*cloud_hull);

    //area = pcl::calculatePolygonArea(*cloud_hull);

    //std::stringstream ss;
    //ss << "cloud_" << area;
    //viewer_.addPointCloud(cloud_proj, ss.str());
    //viewer_.spin();

    return area;
}

void PoseEstimator::getBoundingBox(PointCloudT::Ptr cloud, 
                                   Eigen::Quaternionf& q, 
                                   Eigen::Vector3f& t,
                                   PointNT& maxPt,
                                   PointNT& minPt,
                                   bool rotated /*= true*/)
/* Basic priciple of computing bounding box:
  1) compute the centroid (c0, c1, c2) and the normalized covariance
  2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
  3) move the points in that RF 
     --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
  4) compute the max, the min and the center of the diagonal
  5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) 
     the transformation you have to apply is 
       Rotation = (e0, e1, e0 X e1) & 
       Translation = Rotation * center_diag + (c0, c1, c2)
*/
{
    if (rotated) 
    {
        // compute pricipal direction
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
        eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

        // move the points to the reference frame and compute the size
        Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
        p2w.block<3,3>(0,0) = eigDx.transpose();
        p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
        PointCloudT cPoints;
        pcl::transformPointCloud(*cloud, cPoints, p2w);

        pcl::getMinMax3D(cPoints, minPt, maxPt);
        const Eigen::Vector3f meanDiag = 0.5f * (maxPt.getVector3fMap() + minPt.getVector3fMap());

        // final transform
        q = Eigen::Quaternionf(eigDx);
        t = eigDx * meanDiag + centroid.head<3>();
    } 
    else 
    {
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        t[0] = (maxPt.x + minPt.x) / 2;
        t[1] = (maxPt.y + minPt.y) / 2;
        t[2] = (maxPt.z + minPt.z) / 2;

        q = Eigen::Quaternionf::Identity();
    }

    //std::cout << "Bounding box:\n";
    //std::cout << " - Quaternion: \n" << q.matrix() << std::endl;
    //std::cout << " - Translation: \n" << t << std::endl;
}

void PoseEstimator::loadParameters(std::string filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
    {
        //fs["Method_Type"]                               >> param_.METHOD_TYPE;
        fs["VOXEL_Unit"]                                >> param_.VOXEL_Unit;
        fs["FEAT_NormalEstimateRadius"]                 >> param_.FEAT_NormalEstimateRadius;
        fs["FEAT_FeatureEstimateRadius"]                >> param_.FEAT_FeatureEstimateRadius;
        fs["FEAT_AlignSampleNumber"]                    >> param_.FEAT_AlignSampleNumber;
        fs["FEAT_AlignCorrespondenceRandomness"]        >> param_.FEAT_AlignCorrespondenceRandomness;
        fs["FEAT_AlignSimilarityThreshold"]             >> param_.FEAT_AlignSimilarityThreshold;
        fs["FEAT_AlignMaxCorrespondenceDistanceFactor"] >> param_.FEAT_AlignMaxCorrespondenceDistanceFactor;
        fs["FEAT_AlignInlierFraction"]                  >> param_.FEAT_AlignInlierFraction;
        
        printf("\nLoad matching parameters SUCCEED.\n");
    }
    else
        std::cout << "\nLoad matching parameters FAILED! \n\tFile: " << filename << std::endl;
}

void PoseEstimator::removeSmallCluster(PointCloudT::Ptr srcCloud, PointCloudT::Ptr dstCloud)
{
    int N = srcCloud->points.size();
    int MIN_COUNT = (int)(0.05 * N);

    PointCloudT resCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*srcCloud, *xyzCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (xyzCloud);

    std::vector<pcl::PointIndices> extract_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (2*param_.VOXEL_Unit); 
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (N);
    ec.setSearchMethod (tree);
    ec.setInputCloud (xyzCloud);
    ec.extract (extract_indices);    

    printf("\nRemoving small cluster ... \n  Input cloud have %i cluster\n  MIN_COUNT: %i\n",
        extract_indices.size(), MIN_COUNT);

    for (size_t i = 0; i < extract_indices.size(); i++) 
    {
        std::cout << "  --Cluster size: " << extract_indices[i].indices.size() << std::endl;
        if (extract_indices[i].indices.size() > MIN_COUNT) 
        {
            for (size_t j = 0; j < extract_indices[i].indices.size(); ++j) 
            {
                int id = extract_indices[i].indices[j];
                PointNT pt = srcCloud->points[id];
                resCloud.points.push_back(pt);
            }
        }
    }

    dstCloud->swap(resCloud);
}


//////////////////////////////////////////////////////////////////////////
ImageMatching::ImageMatching(void)
{

}

ImageMatching::~ImageMatching(void)
{

}

float ImageMatching::matchImage(cv::Mat& img1, cv::Mat& img2)
{
    cv::Mat h;
    return matchImage(img1, img2, h);
}

float ImageMatching::matchImage(cv::Mat& img1, cv::Mat& img2, cv::Mat& homography)
{
    float fitness = 0;



    return fitness;
}

bool ImageMatching::extractFeatures(cv::Mat& img, std::vector<cv::KeyPoint>& kps, cv::Mat& des)
{
    cv::SiftFeatureDetector detector_1;
    cv::SiftDescriptorExtractor extractor_1;

    double tkaze = 0.0;
    int64 t1 = cv::getTickCount(), t2 = 0;

    //-- Detect keypoints and calculate descriptors
    detector_1.detect(img, kps);
    extractor_1.compute(img, kps, des);

    t2 = cv::getTickCount();
    tkaze = 1000.0 * (t2 - t1) / cv::getTickFrequency();

    std::cout << "\n-- Image SIFT feature detection time (ms): " << tkaze << std::endl;

    return kps.size() > 4;
}

std::vector<float> ImageMatching::matchImage(cv::Mat& img_1, std::vector<cv::Mat>& db_imgs, std::vector<std::vector<cv::KeyPoint> >& db_kps, std::vector<cv::Mat>& db_des, std::vector<cv::Mat>& homography)
{
    std::vector<float> db_rate;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    extractFeatures(img_1, keypoints_1, descriptors_1);

    
#ifdef DEBUG_MODE0
    float beta = 1;
    int nMatches = db_imgs.size();
    cv::Mat imgMatches;
    int roiHeight = (int)(img_1.rows*beta);

    cv::drawKeypoints(img_1, keypoints_1, img_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif // _DEBUG

    for (size_t k=0; k<db_kps.size(); k++)
    {
        cv::Mat img_2 = db_imgs[k];
        keypoints_2 = db_kps[k];
        descriptors_2 = db_des[k];

        printf("\n-- Keypoint number of img_1 : %d \n", keypoints_1.size() );
        printf("-- Keypoint number of img_2 : %d \n", keypoints_2.size() );

        if (keypoints_1.size() < 4 || keypoints_2.size() < 4)
            continue;

        ////////////////////////////////////////////////////////////////////////////////////
        //-- Matching Keypoints
        cout << "-- Computing homography (RANSAC)..." << endl;
        std::vector<cv::DMatch> matches, inliers;
        cv::Mat H;
        bool filterMatches = true;
        float fitness = 0;

        bfMatch(descriptors_1, descriptors_2, matches, filterMatches);
        fitness = findHomography(keypoints_1, keypoints_2, matches, inliers, H);
        if (0 == inliers.size())
        {
            matches.clear();
            flannMatch(descriptors_1, descriptors_2, matches, filterMatches);
            fitness = findHomography(keypoints_1, keypoints_2, matches, inliers, H);
            if (0 == inliers.size())
            {
                matches.clear();
                knnMatch(descriptors_1, descriptors_2, matches);
                fitness = findHomography(keypoints_1, keypoints_2, matches, inliers, H);                
            }
        }
        float match_rate = inliers.size();

        printf("-- Number of Matches : %d \n", matches.size() );
        printf("-- Number of Inliers : %d \n", inliers.size() );
        printf("-- Match rate : %f \n", matches.size() > 0 ? inliers.size() / (float)matches.size() : 0 );
        printf("-- Fitness : %f \n", fitness );

        db_rate.push_back(match_rate);
#ifdef DEBUG_MODE0
        ////////////////////////////////////////////////////////////////////////////////////
        //-- Draw Keypoints
        drawKeypoints(img_2, keypoints_2, img_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        //-- Draw inliers
        cv::Mat imgMatch;
        cv::drawMatches( img_1, keypoints_1, img_2, keypoints_2,
            inliers, imgMatch, cv::Scalar::all(-1), cv::Scalar::all(-1),
            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners;
        obj_corners.push_back( cv::Point2f(0,0) );
        obj_corners.push_back( cv::Point2f(img_1.cols,0) );
        obj_corners.push_back( cv::Point2f(img_1.cols,img_1.rows) );
        obj_corners.push_back( cv::Point2f(0,img_1.rows) );

        if (!H.empty())
        {
            std::vector<cv::Point2f> scene_corners;
            perspectiveTransform(obj_corners, scene_corners, H);

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            int npts = scene_corners.size();
            for (int i=0; i<npts; i++)
                line( imgMatch, scene_corners[i] + cv::Point2f( img_1.cols, 0), 
                scene_corners[(i+1)%npts] + cv::Point2f( img_1.cols, 0), cv::Scalar(0,70*i,255), 2 );
        }

        //-- Combine all matches
        if (imgMatches.empty())
            imgMatches = cv::Mat( nMatches*roiHeight, (img_1.cols+img_2.cols)*beta, CV_8UC3 );
        cv::Rect roi = cv::Rect(0, k*roiHeight, imgMatches.cols, roiHeight);
        cv::Mat imgRoi = imgMatches(roi);
        resize(imgMatch, imgRoi, imgRoi.size());
        
#endif // _DEBUG
    }

#ifdef DEBUG_MODE0
    cv::imshow("feature", imgMatches);
    std::string png_file = _OUTPUT_DIRECTORY_ "/../../capture/capture_kinect/output/images/" + bin_name_ + "/image_matching_result.png";
    cv::imwrite(png_file, imgMatches);
#endif // DEBUG_MODE0


    return db_rate;
}

// @brief Find homography and inliers
float ImageMatching::findHomography(const std::vector<cv::KeyPoint>& source, const std::vector<cv::KeyPoint>& result, const std::vector<cv::DMatch>& input, std::vector<cv::DMatch>& inliers, cv::Mat& homography)
{
    float fitness = 0;
    inliers.clear();

    if (input.size() < 4)
        return fitness;

    const int pointsCount = input.size();
    const float reprojectionThreshold = 3;

    //Prepare src and dst points
    std::vector<cv::Point2f> srcPoints, dstPoints;    
    for (int i = 0; i < pointsCount; i++)
    {
        srcPoints.push_back(source[input[i].queryIdx].pt);
        dstPoints.push_back(result[input[i].trainIdx].pt);
    }

    // Find homography using RANSAC algorithm
    std::vector<unsigned char> status;
    homography = cv::findHomography(srcPoints, dstPoints, CV_FM_RANSAC, reprojectionThreshold, status);

    if (goodHomography(homography)) 
    {
        // Warp dstPoints to srcPoints domain using inverted homography transformation
        std::vector<cv::Point2f> srcReprojected;
        cv::perspectiveTransform(dstPoints, srcReprojected, homography.inv());

        // Pass only matches with low reprojection error (less than reprojectionThreshold value in pixels)
        inliers.clear();
        for (int i = 0; i < pointsCount; i++)
        {
            cv::Point2f actual = srcPoints[i];
            cv::Point2f expect = srcReprojected[i];
            cv::Point2f v = actual - expect;
            float distanceSquared = v.dot(v);

            if (/*status[i] && */distanceSquared <= reprojectionThreshold * reprojectionThreshold)
            {
                inliers.push_back(input[i]);
                fitness += distanceSquared;
            }
        }
    }

    return fitness;
}

bool ImageMatching::goodHomography(cv::Mat& homography)
{
    bool res = false;

    std::vector<cv::Point2f> srcPt(4), dstPt(4);
    srcPt[0].x = 0; srcPt[0].y = 1;
    srcPt[1].x = 1; srcPt[1].y = 1;
    srcPt[2].x = 1; srcPt[2].y = 0;
    srcPt[3].x = 0; srcPt[3].y = 0;

    cv::perspectiveTransform(srcPt, dstPt, homography);
    cv::Point2f cenPt02, cenPt13;
    cenPt02.x = (dstPt[0].x + dstPt[2].x) / 2;
    cenPt02.y = (dstPt[0].y + dstPt[2].y) / 2;
    cenPt13.x = (dstPt[1].x + dstPt[3].x) / 2;
    cenPt13.y = (dstPt[1].y + dstPt[3].y) / 2;

    if ( 0 < cv::pointPolygonTest(dstPt, cenPt02, false) && 
         0 < cv::pointPolygonTest(dstPt, cenPt13, false) )
        res = true;

    return res;
}

// @brief Use BFMatcher to match descriptors
void ImageMatching::bfMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches, bool filterMatches /* = true */ )
{
    //-- Matching descriptor vectors using Brute-Force matcher
    cout << "--> Use BFMatcher..." << endl;
    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    if (!filterMatches)
    {
        good_matches = matches;
    } 
    else
    {
        double max_dist = 0, min_dist = 100, thresh = 0;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < matches.size(); i++ )
        { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        //thresh = MAX(2*min_dist, min_dist + 0.5*(max_dist - min_dist));
        thresh = 2*min_dist;

        //-- Find initial good matches (i.e. whose distance is less than 2*min_dist )
        for( int i = 0; i < matches.size(); i++ )
        { 
            if( matches[i].distance < thresh )    
            { 
                good_matches.push_back( matches[i]); 
            }
        }
    }
}

// @brief Use FlannBasedMatcher to match descriptors
void ImageMatching::flannMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches, bool filterMatches /* = true */ )
{
    cout << "--> Use FlannBasedMatcher..." << endl;
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    if (!filterMatches)
    {
        good_matches = matches;
    } 
    else
    {
        double max_dist = 0, min_dist = 100, thresh = 0;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < matches.size(); i++ )
        { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        //thresh = MAX(2*min_dist, min_dist + 0.5*(max_dist - min_dist));
        thresh = 2*min_dist;

        //-- Find initial good matches (i.e. whose distance is less than 2*min_dist )
        for( int i = 0; i < matches.size(); i++ )
        { 
            if( matches[i].distance < thresh )    
            { 
                good_matches.push_back( matches[i]); 
            }
        }
    }
}

// @brief Use FlannBasedMatcher with knnMatch to match descriptors
void ImageMatching::knnMatch( cv::Mat& descriptors_1, cv::Mat& descriptors_2, std::vector<cv::DMatch>& good_matches )
{
    cout << "--> Use knnMatch..." << endl;
    std::vector<std::vector<cv::DMatch> > knMatches;
    cv::FlannBasedMatcher matcher;
    int k = 2;
    float maxRatio = 0.75;

    matcher.knnMatch(descriptors_1, descriptors_2, knMatches, k);

    good_matches.clear();

    for (size_t i=0; i< knMatches.size(); i++)
    {
        const cv::DMatch& best = knMatches[i][0];
        const cv::DMatch& good = knMatches[i][1];

        //if (best.distance <= good.distance) continue;

        float ratio = (best.distance / good.distance);
        if (ratio <= maxRatio)
        {
            good_matches.push_back(best);
        }
    }
}
