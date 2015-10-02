#include "RGBD_Capture.h"


#ifdef _IS_LINUX_

RGBDCapture::RGBDCapture(ros::NodeHandle& nh):
    bOpened_ (false),
    bUsePreloadBinDepth_ (true),
    bFlipImage_ (false),
    nMultiGrab_ (5),
    iLowH(DEFAULT_I_LOW_H), iHighH(DEFAULT_I_HIGH_H),
    iLowS(DEFAULT_I_LOW_S), iHighS(DEFAULT_I_HIGH_S),
    iLowV(DEFAULT_I_LOW_V), iHighV(DEFAULT_I_HIGH_V),
    iNearD(DEFAULT_I_FAR_D), iFarD(DEFAULT_I_FAR_D),
    camera_backwall_distance_(0.9)
{
    if (!kinect_.open(CV_CAP_OPENNI)) 
    {
        ROS_ERROR("Can not open a Kinect object.");
        ROS_INFO("Receive Image Service stopped.");
        //return;
    }
    else
    {
        service = nh.advertiseService("receive_image", &RGBDCapture::retrieve, this);
        bOpened_ = true;
        ROS_INFO("Receive Image Service started.");
    }

    loadParameters(_PARAM_DIRECTORY_"/param_capture.yml"); 
    loadCalibResults(_SOURCE_DIRECTORY_"/../../calibration/output/calib_result.yml");
}

bool RGBDCapture::retrieve(RGBD_Capture::SRV_Capture::Request &req, RGBD_Capture::SRV_Capture::Response &res)
{
    bool result = false;
    try {
        int iBlobs = req.blob_number;
        std::string binName = req.bin_name;
        cv::Mat color, depth, mask, pointCloud;

        setBinName(binName);

        result = retrieveImage(color, depth, mask, pointCloud, iBlobs, binName);
        
        //cv::imshow("color", color);
        //cv::imshow("mask", mask);
        
        res.folder = _OUTPUT_DIRECTORY_"/images/" + binName;
        result = imwrite(res.folder + "/rgbImage.png", color);
        result = imwrite(res.folder + "/depthImage.png", depth);
        result = imwrite(res.folder + "/maskImage.png", mask);

        cv::FileStorage fs(std::string(res.folder + "/point_cloud.yml").c_str(), cv::FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "Point Cloud" << pointCloud;
        } else {
            result = false;
        }

        result = copyParamFile(_PARAM_DIRECTORY_"/param_capture.yml", res.folder + "/param.yml");

        for (int i=0;i<3;i++) 
        {
            res.kinect_angle.push_back( kinectAngle_[i] );
        }
    }
    catch (...) {
        result = false;
    }

    res.succeed = result;

    return result;
}

#endif //_IS_LINUX_

RGBDCapture::RGBDCapture(void) :
    bOpened_ (false),
    bUsePreloadBinDepth_ (true),
    bFlipImage_ (false),
    nMultiGrab_ (1),
	binDepth_(float(DEFAULT_BIN_DEPTH_MM)/1000.0),
    iLowH(DEFAULT_I_LOW_H), iHighH(DEFAULT_I_HIGH_H),
    iLowS(DEFAULT_I_LOW_S), iHighS(DEFAULT_I_HIGH_S),
    iLowV(DEFAULT_I_LOW_V), iHighV(DEFAULT_I_HIGH_V),
    iNearD(DEFAULT_I_FAR_D), iFarD(DEFAULT_I_FAR_D),
    camera_backwall_distance_(0.9)
{
    if (!kinect_.open(CV_CAP_OPENNI)) 
    {
        std::cout << "Can not open a Kinect object.\n";        
        //return;
    }
    else
    {
        bOpened_ = true;
    }

    loadParameters(_PARAM_DIRECTORY_"/param_capture.yml");
    loadCalibResults(_SOURCE_DIRECTORY_"/../../calibration/output/calib_result.yml");
}

RGBDCapture::~RGBDCapture(void)
{
    std::cout << "\nCapture node STOPPED!\n\n";
}

bool RGBDCapture::generateMask(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs) 
{
    std::cout << "\n=====>> Generate Mask Started <<=====\n";
    bool result = false;

    //result = getBlobsByRGBD(color, depth, blobs, iBlobs);

    //if (!result)
        result = getBlobsByPointCluster(color, depth, blobs, iBlobs);

    result = filterBlobs(blobs, iBlobs);

    return result;
}

bool RGBDCapture::getBlobsByRGBD(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs)
{
    bool result = false;

    std::string rgb_file = _SOURCE_DIRECTORY_"/../../calibration/output/back_ground_images/"+binName_+"_rgb.png";
    std::string depth_file = _SOURCE_DIRECTORY_"/../../calibration/output/back_ground_images/"+binName_+"_depth.png";

    cv::Mat color_backgound = cv::imread(rgb_file, 1);
    cv::Mat depth_backgound = cv::imread(depth_file, -1);

    if (!color_backgound.empty() && !depth_backgound.empty()) 
    {
        std::cout << "Detecting blobs by extracting foreground...\n";

        cv::Rect roiRect = roiRect_.area() == 0 ? cv::Rect(color.cols/4, color.rows * 0.3, color.cols/2, color.rows/2) : roiRect_;
        std::cout << "ROI: " << roiRect << std::endl;

        cv::Mat color_hsv, bg_hsv, color_h, bg_h, color_diff, depth_diff;
        std::vector<cv::Mat> hsv_color(3), hsv_bg(3);
        
        cv::cvtColor(color(roiRect), color_hsv, CV_BGR2HSV_FULL);
        cv::cvtColor(color_backgound(roiRect), bg_hsv, CV_BGR2HSV_FULL);
        cv::split(color_hsv, hsv_color);
        cv::split(bg_hsv, hsv_bg);
        
        color_h = hsv_color[0];
        bg_h = hsv_bg[0];
        color_diff = cv::abs(hsv_bg[0] - hsv_color[0]) > 15;

        depth_diff = cv::abs(depth_backgound(roiRect) - depth(roiRect)) > 10;

        blobs = color_diff & depth_diff;

        filterBlobs(blobs, iBlobs);
    }

    return result;
}

bool RGBDCapture::getBlobsByPointCluster(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs)
{
    bool result = false;

    if (color.empty() || depth.empty())
        return result;

    std::cout << "Detecting blobs by point clusters...\n";

    // generate ROI cloud
    cv::Rect roiRect = roiRect_.area() == 0 ? cv::Rect(color.cols/4, color.rows * 0.3, color.cols/2, color.rows/2) : roiRect_;
    std::cout << "ROI: " << roiRect << std::endl;

    cv::Mat sub_depth, sub_cloud;    
    cv::Mat roiMask(color.size(), CV_8UC1, cv::Scalar::all(0));

    roiMask(roiRect).setTo(cv::Scalar::all(255));
    depth.copyTo(sub_depth, roiMask);

    VirtualKinectCapture vk;
    vk.generatePointClouds(sub_depth, sub_cloud);

    // detect blobs
    if (!bUsePreloadBinDepth_)
        binSize_[2] = binDepth_;

    blobDetector_.setBinName(binName_);
    blobDetector_.setCameraBackwallDistance(camera_backwall_distance_);
    result = blobDetector_.getBlobs(color, sub_cloud, binSize_, iBlobs, blobs);
    kinectAngle_ = blobDetector_.getKinectAngle();

    return result;
}

bool RGBDCapture::getBlobsByColor(const cv::Mat& color, const cv::Mat& depth, cv::Mat& blobs, int iBlobs)
{
    // http://opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html

    bool result = false;

    if (color.empty())
        return result;

    std::cout << "\nDetecting blobs by HSV information...\n";

    cv::Mat imgSmooth, imgHSV, workingMask, temp_image;

    // generate ROI mask
    if (roiRect_.width <= 0 || roiRect_.height <= 0)
        roiRect_ = cv::Rect(color.cols/4, color.rows/4, color.cols/2, color.rows/2);
    
    cv::Mat roiMask(color.size(), CV_8UC1, cv::Scalar::all(0));
    roiMask(roiRect_).setTo(cv::Scalar::all(255));

    // generate depth mask
    cv::Mat depthMask = depth.clone();
	if (depthMask.rows > 0) { // Only attempt to process depth if it was received as input
		cv::MatIterator_<ushort> it = depthMask.begin<ushort>(), it_end = depthMask.end<ushort>();
		for (; it != it_end; ++it)
		{
			if (*it == 0)
				*it = iNearD;
		}
		inRange(depthMask, cv::Scalar(iNearD), cv::Scalar(iFarD), depthMask);
	}

    // denoise via bilateral filter
    cv::adaptiveBilateralFilter(color, imgSmooth, cv::Size(7, 7), 5, 35);

    // convert the captured frame from BGR to HSV
    cvtColor(imgSmooth, imgHSV, cv::COLOR_BGR2HSV); 

    // threshold the image in HSV colorspace
    inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), workingMask); 

    // merge with roi and depth mask
    workingMask &= roiMask;
    if (depthMask.rows > 0) workingMask &= depthMask; // Only include depth if it was received as input

    blobs = workingMask.clone();

    return result;
}

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = std::abs( cv::contourArea(cv::Mat(contour1)) );
    double j = std::abs( cv::contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

bool RGBDCapture::filterBlobs(cv::Mat& blob_map, int iBlobs)
{
    bool result = false;

	if (blob_map.rows == 0) {
		std::cout << "No blobs in image\n";
		return result;
	}

    std::cout << "Filtering blobs to generate final mask image\n";

    cv::Mat blobs = blob_map.clone();

    // morphological opening (remove small objects)
    erode(blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    dilate( blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

    // morphological closing (fill small holes)
    dilate( blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) ); 
    erode(blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) );

    int blob_count = countBlobs(blob_map);
    if (blob_count < 2 || blob_count > iBlobs) 
    {
        // find all contours in the binary image
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        findContours(blobs, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // sort contours in descending order
        std::sort(contours.begin(), contours.end(), compareContourAreas);

        // erase small blobs
        if (iBlobs < 1) 
            iBlobs = 1;
        while (contours.size() > iBlobs) 
        {
            contours.erase(contours.end() - 1);
        }

        // generate final mask
        double level = 255.0 / double(contours.size());
        if (!contours.empty()) 
        {
            blobs.setTo(cv::Scalar::all(0));
            for (size_t i=0; i<contours.size(); ++i) 
            {
                cv::drawContours(blobs, contours, i, 
                    cv::Scalar((i+1)*level, (i+1)*level, (i+1)*level), 
                    CV_FILLED );
            }
            blob_map = blobs.clone();
        }

        blob_count = contours.size();
    }
    else
    {
        cv::Mat blob_dst;
        blob_map.copyTo(blob_dst, blobs);
        blob_map = blob_dst.clone();
        blob_count = countBlobs(blob_map);
    }

    if (blob_count > 0) 
    {
        printf("The mask image contains %d blobs.\n\n", blob_count);
        result = true;
    }
    else
        printf("The mask image contains ZERO blobs !!!\n\n");

    return result;
}

int RGBDCapture::countBlobs(cv::Mat& blobs)
{
    std::vector<uchar> uniqueLevels;
    cv::MatConstIterator_<uchar> pMask = blobs.begin<uchar>(), pMask_end = blobs.end<uchar>();

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

    return (uniqueLevels.size() - 1);
}

bool RGBDCapture::averagingDepthMaps(std::vector<cv::Mat>& multiDepths, cv::Mat& depth)
{
    VirtualKinect vk;

    return vk.averagingDepthMaps(multiDepths, depth);
}

bool RGBDCapture::loadCalibResults(std::string filename)
{
    bool result = false;

    std::cout << "Loading calibration results from " << filename << std::endl;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["Bin Size"] >> allBinSize_;

        result = true;
    }
    else
        std:: cout << "Fail to load calibration results. Use default value.\n";

    return result;
}

bool RGBDCapture::loadParameters(std::string filename)
{
    bool result = false;

    std::cout << "Loading parameters from " << filename << std::endl;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
    {
        cv::FileNodeIterator fit  = fs["Hue Threshold"].begin();
        fit >> iLowH >> iHighH;
        fit  = fs["Saturation Threshold"].begin();
        fit >> iLowS >> iHighS;
        fit  = fs["Value Threshold"].begin();
        fit >> iLowV >> iHighV;
        fit  = fs["Depth Threshold"].begin();
        fit >> iNearD >> iFarD;

        fs["ROI region"] >> roiRect_;
        fs["Camera Backwall Distance"] >> camera_backwall_distance_;

        if (allBinSize_.empty())
            fs["Bin Size"] >> allBinSize_;

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
}

void RGBDCapture::setBinName(std::string binname)
{ 
    binName_ = binname; 

    std::transform(binname.begin(), binname.end(), binname.begin(), ::tolower);
    char id = binname[binname.length()-1];

    //if (id == 'g' || id == 'j') 
    //{
    //    bFlipImage_ = true;
    //    printf("\nImage is FLIPPED!\n");
    //}
    //else
    //    bFlipImage_ = false;

    id -= 'a';
    int idx = static_cast<int>(id);
    const float *p = allBinSize_.ptr<float>(idx);
    binSize_ = std::vector<float>(p, p + allBinSize_.cols);

    printf("\nCurrent bin for capture: %s\n", binName_.c_str());
    printf("Bin size: %.3f, %.3f, %.3f\n\n", binSize_[0], binSize_[1], binSize_[2]); 
}

bool RGBDCapture::copyParamFile(std::string src, std::string des)
{
    bool result = false;

    try 
    {
#ifdef _IS_LINUX_
        std::ifstream source(src.c_str());
        std::ofstream dest(des.c_str());
#else
        std::ifstream source(src, std::ios::binary);
        std::ofstream dest(des, std::ios::binary);
#endif
        dest << source.rdbuf();

        source.close();
        dest.close();

        result = true;
    }
    catch (...)
    {
    }

    return result;
}

bool RGBDCapture::retrieveImage(cv::Mat& rgbImage, cv::Mat& maskImage, int iBlobs, std::string binName)
{
    cv::Mat depthImage;
    return retrieveImage(rgbImage, depthImage, maskImage, iBlobs, binName);
}

bool RGBDCapture::retrieveImage(cv::Mat& rgbImage, cv::Mat& depthImage, cv::Mat& maskImage, int iBlobs, std::string binName)
{
    pcl::ScopeTime st(__FUNCTION__);

    bool result = false;

    if (!kinect_.isOpened())
        return result;

    if (nMultiGrab_ > 1) 
    {
        std::cout << "Grabing multiple Kinect frames ";

        std::vector<cv::Mat> multiDepths;
        do
        {
            cv::Mat depth;
            if (kinect_.grab()) 
            {
                if (kinect_.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP))
                    multiDepths.push_back(depth);

                if (multiDepths.size() == nMultiGrab_)
                {
                    kinect_.retrieve(rgbImage, CV_CAP_OPENNI_BGR_IMAGE);
                    std::cout << ". ";
                }
            } 
        } 
        while (multiDepths.size() < nMultiGrab_);
        std::cout << "Done.\n";

        averagingDepthMaps(multiDepths, depthImage);
    }
    else
    {
        if( !kinect_.grab() )
        {
            std::cout << "Can not grab images." << std::endl;
        }
        else
        {
            kinect_.retrieve(depthImage, CV_CAP_OPENNI_DEPTH_MAP);
            kinect_.retrieve(rgbImage, CV_CAP_OPENNI_BGR_IMAGE);
        }
    }

    if (bFlipImage_) 
    {
        cv::flip(rgbImage, rgbImage, -1); // -1 means flip the image along both axis (0 for X axis, 1 for Y axis)
        cv::flip(depthImage, depthImage, -1);
    }

    result = generateMask(rgbImage, depthImage, maskImage, iBlobs);

    return result;
}

bool RGBDCapture::retrieveImage(cv::Mat& rgbImage, cv::Mat& depthImage, cv::Mat& maskImage, cv::Mat& pointCloud, int iBlobs, std::string binName)
{
    pcl::ScopeTime st(__FUNCTION__);

    bool result = false;

    if (!kinect_.isOpened())
        return result;

    if (nMultiGrab_ > 1) 
    {
        std::cout << "Grabing multiple Kinect frames ";

        std::vector<cv::Mat> multiDepths;
        do
        {
            cv::Mat depth;
            if (kinect_.grab()) 
            {
                if (kinect_.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP))
                    multiDepths.push_back(depth);

                if (multiDepths.size() == nMultiGrab_)
                {
                    kinect_.retrieve(rgbImage, CV_CAP_OPENNI_BGR_IMAGE);
                    std::cout << ". ";
                }
            } 
        } 
        while (multiDepths.size() < nMultiGrab_);
        std::cout << "Done.\n";

        averagingDepthMaps(multiDepths, depthImage);
    }
    else
    {
        if( !kinect_.grab() )
        {
            std::cout << "Can not grab images." << std::endl;
        }
        else
        {
            kinect_.retrieve(depthImage, CV_CAP_OPENNI_DEPTH_MAP);
            kinect_.retrieve(rgbImage, CV_CAP_OPENNI_BGR_IMAGE);
            //kinect_.retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        }
    }

    if (bFlipImage_) 
    {
        cv::flip(rgbImage, rgbImage, -1); // -1 means flip the image along both axis (0 for X axis, 1 for Y axis)
        cv::flip(depthImage, depthImage, -1);
    }

    VirtualKinectCapture vk;
    vk.generatePointClouds(depthImage, pointCloud);

    result = generateMask(rgbImage, depthImage, maskImage, iBlobs);

    return result;
}

//////////////////////////////////////////////////////////////////////////

BinBlobsDetector::BinBlobsDetector()
    : cloud_ (new CloudT)
{
    camera_model_.points.resize(5);
    float z = 0.15, w, h;
    float aw = DEFAULT_DEPTH_FOV_H/2, ah = DEFAULT_DEPTH_FOV_V/2;
    w = std::tan(aw) * z;
    h = std::tan(ah) * z;
    camera_model_.points[0] = PointT(0,0,0);
    camera_model_.points[1] = PointT( w,  h, -z);
    camera_model_.points[2] = PointT( w, -h, -z);
    camera_model_.points[3] = PointT(-w, -h, -z);
    camera_model_.points[4] = PointT(-w,  h, -z);

    viewer_.setWindowName("BinBlobsDetector");
}

BinBlobsDetector::~BinBlobsDetector()
{
    viewer_.close();
}


void BinBlobsDetector::init(const cv::Mat& color, cv::Mat& cloud)
{
    color_ = color.clone();
    copyCloud(cloud, cloud_);
}

bool BinBlobsDetector::getBlobs(const cv::Mat& color, cv::Mat& cloud, std::vector<float> bin_size, int iBlobs, cv::Mat& blobs)
{
    bool result = false;

    try 
    {      
        found_back_wall_ = false;
        found_front_bar_ = false;
        bin_width_ = bin_size[0];
        bin_height_= bin_size[1];
        bin_depth_ = bin_size[2];

#if PCL_VERSION_COMPARE(>,1,7,2)
        viewer_.removeAllCoordinateSystems();
#else
        viewer_.removeCoordinateSystem();
#endif 
        viewer_.removeAllPointClouds();
        viewer_.removeAllShapes();
        viewer_.addCoordinateSystem(0.15);

        init(color, cloud);

        std::vector<CloudT> clusters; 
        std::vector<std::vector<int> > cluster_indices;

        getClusters(cloud_, clusters, cluster_indices);
        getClusterInBin(clusters, cluster_indices, blobs);
        
        result = true;
    }
    catch (...)
    {
        result = false;
    }

    return result;
}

// TODO: use RGB info to filter bars and walls
void BinBlobsDetector::copyCloud(cv::Mat& cvCloud, CloudT::Ptr dstCloud)
{
    pcl::ScopeTime st(__FUNCTION__);

    dstCloud->clear();
    mapPointToImage_.clear();

    CloudT::Ptr pclCloud (new CloudT);

    for(int r=0; r<cvCloud.rows; r++)
    {
        cv::Point3f* cloud_ptr = (cv::Point3f*)cvCloud.ptr(r);
        for(int c=0; c<cvCloud.cols; c++)
        {
            if (cloud_ptr[c].z > 0)
            {
                PointT pt;
                pt.x = cloud_ptr[c].x;
                pt.y = cloud_ptr[c].y;
                pt.z = -cloud_ptr[c].z;

                pclCloud->points.push_back(pt);

                int idx = r * cvCloud.cols + c;
                mapPointToImage_.push_back(idx);
            } 
        }
    }

    adjustCloud(pclCloud, dstCloud);
}

void BinBlobsDetector::adjustCloud(CloudT::Ptr srcCloud, CloudT::Ptr dstCloud)
{
    pcl::ScopeTime st(__FUNCTION__);

    // Down sampling
    CloudT::Ptr vgCloud (new CloudT() ), bwCloud (new CloudT() );
    const float leaf = 0.01; // Unit: meter

    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize (leaf, leaf, leaf);
    vg.setInputCloud (srcCloud);
    vg.filter (*vgCloud);

    // Get points near back wall of the bin
    float z_min = -camera_backwall_distance_ - 0.1;
    float z_max = -camera_backwall_distance_ + 0.1;
    pcl::PassThrough<PointT> pass(true); // allow to extract removed list
    pass.setInputCloud (vgCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*bwCloud);
    printf("Extracting points between %f - %f meters along Z axis\n", z_min, z_max);
    std::cout << "Candidate backwall points: " << bwCloud->points.size() << std::endl;

    // Estimate plane
    pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud(bwCloud);
    seg.segment(*inliers, *plane_coeff);

    // Adjust cloud
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q;
    Eigen::Vector3f rot_axis, rot_angle;
    Eigen::Vector3f orie_ref(0,0,1); // Z axis
    Eigen::Vector3f orie_plane(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);

    float theta = std::acos( orie_plane.dot(orie_ref) );
    rot_axis = (orie_ref.cross(orie_plane)).normalized();
    q = Eigen::AngleAxisf( -theta, rot_axis );
    t = q.normalized();

    vgCloud->clear();
    pcl::transformPointCloud(*srcCloud, *vgCloud, t.matrix());

    pass.setInputCloud (vgCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, 0);
    pass.filter (*dstCloud);

    const pcl::IndicesConstPtr &indices = pass.getRemovedIndices();
    for (int i=0;i<indices->size();i++) 
    {
        int id = indices->at(i);
        mapPointToImage_[id] = -1;
    }
    for (int i = 0; i < mapPointToImage_.size(); i++)
    {
        if (mapPointToImage_[i] < 0) 
        {
            mapPointToImage_.erase(mapPointToImage_.begin() + i);
            i--;
        }
    }

    pcl::getEulerAngles(t, rot_angle[0], rot_angle[1], rot_angle[2]);
    kinect_angle_ = rot_angle;

    std::cout << "Rotating the point cloud to align with Z axis. \n";
    printf(" Rotate angle: %f, %f, %f\n", RAD2DEG(rot_angle[0]), RAD2DEG(rot_angle[1]), RAD2DEG(rot_angle[2])); 
    printf(" Kinect angle: %f, %f, %f\n", RAD2DEG(kinect_angle_[0]), RAD2DEG(kinect_angle_[1]), RAD2DEG(kinect_angle_[2])); 

    Eigen::Affine3f t_k (t.matrix());
    drawCameraModel(t_k);
}

void BinBlobsDetector::drawCameraModel(Eigen::Affine3f tk)
{
    CloudT::Ptr camera_model_aligned (new CloudT);
    pcl::transformPointCloud(camera_model_, *camera_model_aligned, tk);

    for (int i=0; i<4; i++) 
    {
        PointT pt0 = camera_model_aligned->points[0];
        PointT pt1 = camera_model_aligned->points[i+1];
        PointT pt2 = camera_model_aligned->points[(i+1)%4+1];
        std::stringstream ss;

        ss << "cam_line_" << i;
        viewer_.addLine(pt0, pt1, 1,1,0, ss.str());
        ss << i;
        viewer_.addLine(pt2, pt1, 1,1,0, ss.str());
    }
}

void BinBlobsDetector::findBackwallAndFrontBar(std::vector<pcl::ModelCoefficients> coeffs)
{
    pcl::ScopeTime st(__FUNCTION__);

    float min_dist = 9e9, max_dist = -9e9;
    int min_id = -1, max_id = -1;
    float BACKWALL_DIST = camera_backwall_distance_, BACKWALL_DIST_THRESH = 0.1f;
    float FRONTBAR_DIST = BACKWALL_DIST - bin_depth_, FRONTBAR_DIST_THRESH = 0.1f;
    float THETA_THRESH = 5.0 * M_PI / 180.0;

    for (size_t i = 0; i < coeffs.size(); ++i) 
    {
        PointT camera (0,0,0);
        float cam_plane_dist = 0;
        cam_plane_dist = pcl::pointToPlaneDistance(camera, coeffs[i].values[0], coeffs[i].values[1], coeffs[i].values[2], coeffs[i].values[3]);
        //cam_plane_dist = std::abs(coeffs[i].values[3]);
        
        Eigen::Vector3f orie_z(1,0,0); // Z axis
        Eigen::Vector3f orie_plane(coeffs[i].values[0], coeffs[i].values[1], coeffs[i].values[2]);
        float theta = std::acos( orie_plane.dot(orie_z) );

        if (std::abs(theta - M_PI_2) < THETA_THRESH) 
        {
            if (max_dist < cam_plane_dist && 
                std::abs(cam_plane_dist - BACKWALL_DIST) < BACKWALL_DIST_THRESH) 
            {
                max_dist = cam_plane_dist;
                max_id = i;
            }
            if (min_dist > cam_plane_dist && 
                std::abs(cam_plane_dist - FRONTBAR_DIST) < FRONTBAR_DIST_THRESH) 
            {
                min_dist = cam_plane_dist;
                min_id = i;
            }
        }

        std::cout << "Plane <-> Z angle: " << RAD2DEG(theta) << std::endl;
        std::cout << "Distance from camera to plane: " << cam_plane_dist << std::endl;
    }

    if (max_id != -1) 
    {
        found_back_wall_ = true;

        back_wall_coeff_[0] = coeffs[max_id].values[0];
        back_wall_coeff_[1] = coeffs[max_id].values[1];
        back_wall_coeff_[2] = coeffs[max_id].values[2];
        back_wall_coeff_[3] = coeffs[max_id].values[3];
    }
    if (min_id != -1) 
    {
        found_front_bar_ = true;

        front_bar_coeff_[0] = coeffs[min_id].values[0];
        front_bar_coeff_[1] = coeffs[min_id].values[1];
        front_bar_coeff_[2] = coeffs[min_id].values[2];
        front_bar_coeff_[3] = coeffs[min_id].values[3];

        printf("Found front bar -\n Front bar coefficent: [%f, %f, %f, %f]\n",
            front_bar_coeff_[0], front_bar_coeff_[1],
            front_bar_coeff_[2], front_bar_coeff_[3]);
    }
}

void BinBlobsDetector::extractBackWall()
{
    pcl::ScopeTime st(__FUNCTION__);

    // If the back wall plane contains multiple part, 
    // choose the one near to the estimated pin center
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (back_wall_.makeShared());

    std::vector<pcl::PointIndices> extract_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 1cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (back_wall_.makeShared());
    ec.extract (extract_indices);    
    
    std::cout << "Number of candidate back walls: " << extract_indices.size() << std::endl;

    CloudT::Ptr central_part (new CloudT);
    float min_dist = 1e10;
    int min_id = 0;
    PointT center_pt (0, -bin_height_ / 2, 0);
    
    for (size_t k=0; k<extract_indices.size(); k++)
    {
        float x_mean = 0, y_mean = 0;
        size_t N  = extract_indices[k].indices.size();
        for (size_t i=0; i<N; i++)
        {
            int id = extract_indices[k].indices[i];
            float dx = back_wall_.points[id].x - center_pt.x;
            float dy = back_wall_.points[id].y - center_pt.y;
            x_mean += dx * dx;
            y_mean += dy * dy;
        }
        x_mean /= N;
        y_mean /= N;
        x_mean = x_mean*x_mean + y_mean*y_mean;

        if (x_mean < min_dist)
        {
            min_dist = x_mean;
            min_id = k;
        }
    }

    size_t M = extract_indices[min_id].indices.size();
    central_part->points.resize( M );
    for (size_t i=0; i<M; i++)
    {
        int id = extract_indices[min_id].indices[i];
        central_part->points[i] = back_wall_.points[id];
    }
    central_part->width = M;
    central_part->height = 1;
    central_part->is_dense = true;

    int MIN_CENTER_DIST = bin_width_*0.45;
    MIN_CENTER_DIST *= MIN_CENTER_DIST;

    if (M > 20 /*&& min_dist < MIN_CENTER_DIST*/)
    {
        back_wall_.swap(*central_part);

        pcl::getMinMax3D(back_wall_, back_wall_min_pt_, back_wall_max_pt_);

        //back_wall_min_pt_.x = MIN(back_wall_min_pt_.x, -0.1);
        //back_wall_max_pt_.x = MAX(back_wall_max_pt_.x, 0.1);
        //back_wall_min_pt_.y = MIN(back_wall_min_pt_.y, back_wall_max_pt_.y - 0.2);

        found_back_wall_ = true;

        std::cout << "Found back wall - \n";
    } 
    else 
    {
        found_back_wall_ = false;
        back_wall_coeff_ = Eigen::Vector4f(0,0,1,1);
        back_wall_max_pt_.x = bin_width_ / 2;
        back_wall_max_pt_.y = 0;
        back_wall_max_pt_.z = -1;
        back_wall_min_pt_.x = -bin_width_ / 2;
        back_wall_min_pt_.y = -bin_height_;
        back_wall_min_pt_.z = -1;

        std::cout << "FAILED to find Back Wall !!! Use default parameters:\n";
    }

    printf(" Back wall coefficent: [%f, %f, %f, %f]\n",
        back_wall_coeff_[0], back_wall_coeff_[1],
        back_wall_coeff_[2], back_wall_coeff_[3]);
    std::cout << "  Max point: " << back_wall_max_pt_;
    std::cout << "\n  Min point: " << back_wall_min_pt_ << std::endl;
}

void BinBlobsDetector::voxelize(CloudT src, CloudT::Ptr dst, std::vector<std::vector<int> >& indices)
{
    pcl::ScopeTime st(__FUNCTION__);

    pcl::PointXYZ ptMax, ptMin;
    pcl::getMinMax3D(src, ptMin, ptMax);

    float g_fVoxelUnit = 0.005;
    int g_iVoxelCount_X = static_cast<int>((ptMax.x - ptMin.x)/g_fVoxelUnit) + 1;
    int g_iVoxelCount_Y = static_cast<int>((ptMax.y - ptMin.y)/g_fVoxelUnit) + 1;
    int g_iVoxelCount_Z = static_cast<int>((ptMax.z - ptMin.z)/g_fVoxelUnit) + 1;

    int vgSize = g_iVoxelCount_X * g_iVoxelCount_Y * g_iVoxelCount_Z;
    int vcXY = g_iVoxelCount_X * g_iVoxelCount_Y;

    std::vector<int> m_vecValidVoxelID(vgSize, -1);
    int m_iPointsCount = src.points.size();

    int m_iVoxelPointsCount = 0;

    dst->clear();
    indices.clear();

    //#pragma omp parallel for
    for(int p = 0; p < m_iPointsCount; p++)
    {
        float pos_X = src.points[p].x;
        float pos_Y = src.points[p].y;
        float pos_Z = src.points[p].z;

        int ipos_Z = static_cast<int>((pos_Z - ptMin.z) / g_fVoxelUnit),
            ipos_Y = static_cast<int>((pos_Y - ptMin.y) / g_fVoxelUnit),
            ipos_X = static_cast<int>((pos_X - ptMin.x) / g_fVoxelUnit);
        int id = ipos_Z * vcXY + ipos_Y * g_iVoxelCount_X + ipos_X;
        //if (id >= m_vecValidVoxelID.size())
        //    id = m_vecValidVoxelID.size() - 1;

        bool isInNewVoxel = m_vecValidVoxelID[id] < 0;
        bool isInOldVoxel = m_vecValidVoxelID[id] >= 0 && m_vecValidVoxelID[id] < m_iVoxelPointsCount;

        if ( isInNewVoxel )
        {
            m_vecValidVoxelID[id] = m_iVoxelPointsCount;

            pcl::PointXYZ pt;
            pt.x = pos_X;
            pt.y = pos_Y;
            pt.z = pos_Z;

            dst->points.push_back(pt);
            std::vector<int> newIndex;
            newIndex.push_back(p);
            indices.push_back(newIndex);

            m_iVoxelPointsCount++;
        }
        else
        {
            int ind = m_vecValidVoxelID[id];
            indices[ind].push_back( p );
        }
    }
}

void BinBlobsDetector::getClusters(CloudT::Ptr cloud, std::vector<CloudT>& clusters, std::vector<std::vector<int> >& cluster_indices)
{
    pcl::ScopeTime st(__FUNCTION__);

    CloudT::Ptr cloud_v0(new CloudT), cloud_f (new CloudT), cloud_v (new CloudT);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    std::vector<std::vector<int> > voxel_indices;
    voxelize(*cloud, cloud_v, voxel_indices);
    pcl::copyPointCloud(*cloud_v, *cloud_v0);
    //std::cout << "PointCloud after filtering has: " << cloud_f->points.size ()  << " data points." << std::endl; 
    
    // Create the segmentation object for the planar model 
    printf("\nExtract ALL planar surface and find the correct backwall and front bar\n");

    std::vector<pcl::ModelCoefficients> plane_coeffs;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    
    int nr_points = (int) cloud_v->points.size ();

    while (cloud_v->points.size () > 0.1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setInputCloud (cloud_v);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_v);
        extract.setIndices (inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_v = *cloud_f;

        plane_coeffs.push_back(*coefficients);
    }

    findBackwallAndFrontBar(plane_coeffs);


    if (found_back_wall_) 
    {
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr scmp (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_v0)); 

        float thresh = 0.02f;
        std::vector<int> inliers; 
        scmp -> selectWithinDistance (back_wall_coeff_, thresh, inliers); 
        pcl::copyPointCloud(*cloud_v0, inliers, back_wall_); 

        extractBackWall();
    }

    getPointsInBin(*cloud_v0, cloud_v, voxel_indices);

    viewer_.addPointCloud(cloud_v0);
    viewer_.addPointCloud(back_wall_.makeShared(), "back_wall");
    viewer_.addPointCloud(cloud_v, pcl::visualization::PointCloudColorHandlerCustom<PointT> (cloud_v, 0.0, 255.0, 0.0), "cloud_in_bin");

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_v);

    cluster_indices.clear();

    int CLUSTER_SIZE_THRESH = static_cast<int>(0.005 * cloud->points.size());
    std::vector<pcl::PointIndices> extract_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 1cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_v);
    ec.extract (extract_indices);    
    
    std::cout << "Removing small cluster of which the size less than " << CLUSTER_SIZE_THRESH << " points\n";

    for (std::vector<pcl::PointIndices>::const_iterator it = extract_indices.begin (); it != extract_indices.end (); ++it)
    {
        CloudT::Ptr cloud_cluster (new CloudT);
        std::vector<int> cloud_indices;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            for (size_t k = 0; k < voxel_indices[*pit].size(); k++) 
            {
                int id = voxel_indices[*pit][k];

                PointT pt;
                pt = cloud->points[id];

                cloud_cluster->points.push_back(pt);
                cloud_indices.push_back(id);
            }
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if (cloud_cluster->size() > CLUSTER_SIZE_THRESH) 
        {
            clusters.push_back(*cloud_cluster);
            cluster_indices.push_back(cloud_indices);      
        }
    }

    printf("%d out of %d clusters are inside current bin\n", clusters.size(), extract_indices.size());
}

void BinBlobsDetector::getPointsInBin(CloudT src, CloudT::Ptr dst, std::vector<std::vector<int> >& index)
{
    pcl::ScopeTime st(__FUNCTION__);

    float BW = -pointToPlaneDistance(pcl::PointXYZ(0,0,0), back_wall_coeff_);
    float MAGIN = 0.01f; // Unit: meter
    float X_MIN = -bin_width_ * 0.5 + MAGIN;
    float X_MAX = X_MIN + bin_width_ - MAGIN;
    float Y_MAX = back_wall_max_pt_.y; 
    float Y_MIN = MIN(back_wall_min_pt_.y, Y_MAX - bin_height_);
    float Z_MIN = MAGIN;
    float Z_MAX = bin_depth_ - MAGIN*3;

    if ((back_wall_max_pt_.x - back_wall_min_pt_.x) > 0.9*bin_width_) 
    {
        X_MIN = back_wall_min_pt_.x + MAGIN;
        X_MAX = back_wall_max_pt_.x - MAGIN;
    }

    float X_MAX2 = X_MAX - MAGIN*2;
    float X_MIN2 = X_MIN + MAGIN*2;
    float Y_MAX2 = Y_MAX - MAGIN*2;
    float Y_MIN2 = Y_MIN + MAGIN*2;
    float Z_MAX2 = bin_depth_ + MAGIN*3;

    // compute the front window of the bin
    {
        CloudT::Ptr fb_cloud (new CloudT), sub_cloud (new CloudT), sub_cloud2 (new CloudT);
        PointT minPt, maxPt, cenPt(0, Y_MAX/2+Y_MIN/2, Z_MAX);
        float fb_z_min = BW + bin_depth_ - MAGIN*2, fb_z_max = BW + Z_MAX2;

        // front bar cloud
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (src.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (fb_z_min, fb_z_max);
        pass.filter (*fb_cloud);

        // left side wall
        float fb_y_min = cenPt.y - MAGIN*1, fb_y_max = cenPt.y + MAGIN*1;
        float fb_x_min = X_MIN - MAGIN*5, fb_x_max = X_MIN + MAGIN*5;
        pass.setInputCloud(fb_cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (fb_x_min, fb_x_max);
        pass.filter (*sub_cloud2);
        pass.setInputCloud(sub_cloud2);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (fb_y_min, fb_y_max);
        pass.filter (*sub_cloud);
        if (sub_cloud->points.size() > 0) 
        {
            pcl::getMinMax3D(*sub_cloud, minPt, maxPt);
            X_MIN2 = maxPt.x + MAGIN;
        }

        // right side wall
        fb_x_min = X_MAX - MAGIN*5, fb_x_max = X_MAX + MAGIN*5;
        pass.setInputCloud(fb_cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (fb_x_min, fb_x_max);
        pass.filter (*sub_cloud2);
        pass.setInputCloud(sub_cloud2);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (fb_y_min, fb_y_max);
        pass.filter (*sub_cloud);
        if (sub_cloud->points.size() > 0) 
        {
	        pcl::getMinMax3D(*sub_cloud, minPt, maxPt);
	        X_MAX2 = minPt.x - MAGIN;
        }

        // top bar
        fb_x_min = cenPt.x - MAGIN*1, fb_x_max = cenPt.x + MAGIN*1;
        fb_y_min = Y_MAX - MAGIN*10, fb_y_max = Y_MAX + MAGIN*10;
        fb_z_min = BW + bin_depth_ - MAGIN, fb_z_max = BW + Z_MAX2;
        pass.setInputCloud (src.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (fb_z_min, fb_z_max);
        pass.filter (*fb_cloud);
        pass.setInputCloud(fb_cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (fb_x_min, fb_x_max);
        pass.filter (*sub_cloud2);
        pass.setInputCloud(sub_cloud2);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (fb_y_min, fb_y_max);
        pass.filter (*sub_cloud);
        if (sub_cloud->points.size() > 0) 
        {
	        pcl::getMinMax3D(*sub_cloud, minPt, maxPt);
	        Y_MAX2 = minPt.y - MAGIN;
        }

        // bottom bar
        fb_y_min = Y_MIN - MAGIN*10, fb_y_max = Y_MIN + MAGIN*10;
        pass.setInputCloud(fb_cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (fb_x_min, fb_x_max);
        pass.filter (*sub_cloud2);
        pass.setInputCloud(sub_cloud2);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (fb_y_min, fb_y_max);
        pass.filter (*sub_cloud);
        if (sub_cloud->points.size() > 0) 
        {
	        pcl::getMinMax3D(*sub_cloud, minPt, maxPt);
	        Y_MIN2 = maxPt.y + MAGIN;
        }
    }

    printf("\nBin depth: %.3f\nBin space:\n %.3f, %.3f, %.3f\n %.3f, %.3f, %.3f\n %.3f, %.3f, %.3f\n\n",
        bin_depth_,
        X_MIN, X_MAX, X_MAX-X_MIN,
        Y_MIN, Y_MAX, Y_MAX-Y_MIN,
        BW+Z_MIN, BW+Z_MAX, Z_MAX-Z_MIN);

    viewer_.addCube(X_MIN, X_MAX, Y_MIN, Y_MAX, BW+Z_MIN, BW+Z_MAX, 0, 1, 0, "cube1");
    viewer_.addCube(X_MIN2, X_MAX2, Y_MIN2, Y_MAX2, BW+Z_MAX, BW+Z_MAX2, 1, 0, 0, "cube2");

    std::vector<int> pointsInBin; 
    std::vector<std::vector<int> > newIndex;
    for (size_t id=0; id<src.points.size(); ++id) 
    {
        PointT pt = src.points[id];
        float dist_backwall = pointToPlaneDistanceSigned(pt, back_wall_coeff_);
        int inBin = 0;

        if (dist_backwall < Z_MAX) 
        {
            if (X_MIN < pt.x && pt.x < X_MAX && 
                Y_MIN < pt.y && pt.y < Y_MAX &&
                Z_MIN < dist_backwall) 
            {
                pointsInBin.push_back(id);
            }
        } 
        else if (dist_backwall < Z_MAX2)
        {
            if (X_MIN2 < pt.x && pt.x < X_MAX2 && 
                Y_MIN2 < pt.y && pt.y < Y_MAX2 &&
                Z_MIN < dist_backwall) 
            {
                pointsInBin.push_back(id);
            }
        }

    }

    dst->clear();
    dst->points.resize( pointsInBin.size() );
    newIndex.resize( pointsInBin.size() );
    for (size_t i=0; i<pointsInBin.size(); ++i)
    {
        int id = pointsInBin[i];
        dst->points[i] = src.points[id];
        newIndex[i] = index[id];
    }

    newIndex.swap(index);
}

void BinBlobsDetector::getClusterInBin(std::vector<CloudT>& clusters, std::vector<std::vector<int> >& cluster_indices, cv::Mat& blobs)
{
    pcl::ScopeTime st(__FUNCTION__);

    blobs = cv::Mat::zeros(color_.size(), CV_8UC1);
    uchar* blob_ptr = blobs.ptr<uchar>(0);
    int blob_count = 0;
    
    for (size_t i = 0; i < clusters.size(); i++) 
    {
        blob_count++;
        for (size_t j = 0; j < clusters[i].size(); j++) 
        {
            int pid = cluster_indices[i][j];
            int mid = mapPointToImage_[pid];
            if (mid >= 0)
                blob_ptr[mid] = blob_count;
        }  
    }

    if (blob_count > 0) 
    {
        int alpha = 255 / blob_count;
        blobs *= alpha;
    }

    viewer_.setCameraPosition(
        1.77611, 0.310963, 0.912488, 
        0.0578379, -0.110827, -0.537241, 
        -0.098732, 0.980774, -0.168329);

    viewer_.spinOnce();

    std::string png_file = _OUTPUT_DIRECTORY_ "/images/" + bin_name_ + "/capture_result.png"; 
    viewer_.saveScreenshot(png_file); 

}
