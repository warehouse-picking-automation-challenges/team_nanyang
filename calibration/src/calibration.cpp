#include "calibration.h"

MarkerBasedCalibration::MarkerBasedCalibration(void) :
    iLowH(DEFAULT_I_LOW_H), iHighH(DEFAULT_I_HIGH_H),
    iLowS(DEFAULT_I_LOW_S), iHighS(DEFAULT_I_HIGH_S),
    iLowV(DEFAULT_I_LOW_V), iHighV(DEFAULT_I_HIGH_V),
    iNearD(DEFAULT_I_FAR_D), iFarD(DEFAULT_I_FAR_D),
    SIZE_HORI_WALL_(5), SIZE_VERT_WALL_(4)
{
    loadParameters(_PARAM_DIRECTORY_"/param_calibration.yml");
    initWindow();
}

MarkerBasedCalibration::~MarkerBasedCalibration(void)
{
    saveParameters(_PARAM_DIRECTORY_"/param_calibration.yml");
    if (marker_coordinates_.size() == MARKER_COUNT_)
        saveResults(_OUTPUT_DIRECTORY_"/calib_result.yml");
    viewer_.close();
    cv::destroyAllWindows();
}

void MarkerBasedCalibration::initWindow()
{
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    cvCreateTrackbar("NearD", "Control", &iNearD, 3000);
    cvCreateTrackbar("FarD", "Control", &iFarD, 3000);

    camera_model_.points.resize(5);
    float z = 0.15, w, h;
    float aw = DEG2RAD( 58 / 2 ), ah = DEG2RAD( 45 / 2 );
    w = std::tan(aw) * z;
    h = std::tan(ah) * z;
    camera_model_.points[0] = PointT(0,0,0);
    camera_model_.points[1] = PointT( w,  h, -z);
    camera_model_.points[2] = PointT( w, -h, -z);
    camera_model_.points[3] = PointT(-w, -h, -z);
    camera_model_.points[4] = PointT(-w,  h, -z);
}

void MarkerBasedCalibration::hsvSpectrum()
{
    static int W = 360, H = 100;
    static cv::Mat mat_h, mat_v;
    if (mat_h.empty()) 
    {
        mat_h.create(H, W, CV_8UC1);
        mat_v.create(H, W, CV_8UC1);
        for (int r=0;r<mat_h.rows;r++) 
        {
            for (int c=0;c<mat_h.cols;c++) 
            {
                mat_h.at<uchar>(r,c) = (int)(180.0*c/mat_h.cols);
                mat_v.at<uchar>(r,c) = (int)(255-255.0*r/mat_h.rows);
            }
        }
    }
    
    cv::Mat mat_sl(H, W, CV_8UC1), mat_sh(H, W, CV_8UC1);
    for (int r=0;r<mat_sl.rows;r++) 
    {
        for (int c=0;c<mat_sl.cols;c++) 
        {
            mat_sl.at<uchar>(r,c) = iLowS;
            mat_sh.at<uchar>(r,c) = iHighS;
        }
    }

    cv::Mat mat_hsv1, mat_hsv2;
    std::vector<cv::Mat> hsv1(3), hsv2(3);
    hsv1[0] = hsv2[0] = mat_h;
    hsv1[2] = hsv2[2] = mat_v;
    hsv1[1] = mat_sl;
    hsv2[1] = mat_sh;
    cv::merge(hsv1, mat_hsv1);
    cv::merge(hsv2, mat_hsv2);

    cv::Mat spectrum(H*2+10, W, CV_8UC3);
    spectrum = cv::Scalar::all(150);

    cv::Rect roi(0,0,W, H);
    cv::Mat imRoi(spectrum, roi);
    cv::cvtColor(mat_hsv1, imRoi, CV_HSV2BGR);

    float uH = W/180.0, uV = H/255.0;
    roi.x = iLowH*uH;
    roi.y = iLowV*uV;
    roi.width = (iHighH-iLowH)*uH;
    roi.height = (iHighV-iLowV)*uV;
    cv::rectangle(imRoi, roi, CV_RGB(255,0,0), 2);

    roi = cv::Rect(0,H+10,W,H);
    imRoi = cv::Mat(spectrum, roi);
    cv::cvtColor(mat_hsv2, imRoi, CV_HSV2BGR);

    roi.x = iLowH*uH;
    roi.y = iLowV*uV;
    roi.width = (iHighH-iLowH)*uH;
    roi.height = (iHighV-iLowV)*uV;
    cv::rectangle(imRoi, roi, CV_RGB(255,0,0), 2);

    cv::imshow("Control", spectrum);
}

bool MarkerBasedCalibration::processFrame(cv::Mat& rgb, cv::Mat& depth, cv::Mat& cloud,  bool newFrame /* = true */)
{
    bool result = false;

    new_frame_ = newFrame;
    color_ = rgb.clone();
    depth_ = depth.clone();
    cloud_ = cloud.clone();
    pcl_cloud_ = ColorCloudT::Ptr(new ColorCloudT());
    copyCloud(color_, cloud_, pcl_cloud_);

    if (new_frame_) 
    {
        found_back_wall_ = false;
        found_front_bar_ = false;
        bin_depth_ = 0;
        marker_coordinates_.clear();
        grid_points_.clear();
        bar_centers_.clear();

        found_markers_ = getMarkers(color_, depth_, MARKER_COUNT_);

        if (found_markers_) 
        {            
            result = estimateShelfDimensions();
        }
    }
    else
    {
        result = true;
    }

    showResults();

    return result;
}

bool MarkerBasedCalibration::getMarkers(const cv::Mat& color, const cv::Mat& depth, int iMarkers)
{
    // http://opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html

    bool result = false;

    if (color.empty())
        return result;

    std::cout << "\nDetecting markers by HSV information...\n";

    cv::Mat imgSmooth, imgHSV, workingMask, temp_image;    

    // generate depth mask
    cv::Mat depthMask = depth.clone();
    if (depthMask.rows > 0) { // Only attempt to process depth if it was received as input
        //cv::MatIterator_<ushort> it = depthMask.begin<ushort>(), it_end = depthMask.end<ushort>();
        //for (; it != it_end; ++it)
        //{
        //    if (*it == 0)
        //        *it = iNearD;
        //}
        inRange(depthMask, cv::Scalar(iNearD), cv::Scalar(iFarD), depthMask);
    }

    // denoise via bilateral filter
    cv::adaptiveBilateralFilter(color, imgSmooth, cv::Size(7, 7), 5, 35);

    // convert the captured frame from BGR to HSV
    cvtColor(imgSmooth, imgHSV, cv::COLOR_BGR2HSV); 

    // threshold the image in HSV colorspace
    inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), workingMask); 

    // merge with roi and depth mask
    if (depthMask.rows > 0) workingMask &= depthMask; // Only include depth if it was received as input

    result = filterMarkers(workingMask, iMarkers);

    return result;
}

bool MarkerBasedCalibration::filterMarkers(cv::Mat& blobs, int iBlobs)
{
    bool result = false; 

    // morphological opening (remove small objects)
    erode(blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    dilate( blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

    // morphological closing (fill small holes)
    dilate( blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    erode(blobs, blobs, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
       
    /*
    // Detect circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blobs, circles, CV_HOUGH_GRADIENT, 1, blobs.cols/10, 150, 50);

    std::cout << "Find " << circles.size() << " circles. \n";

    // Draw and validate the circles detected
    cv::Mat imCircles = color_.clone();   
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( imCircles, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( imCircles, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        
        // validate circle as marker
        validateMarkers(depth_, center, radius);
    }
    imshow("circles", imCircles);
    */
    
    // Find all contours in the binary image
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat blobCopy = blobs.clone(), colorMask = cv::Mat::zeros(color_.size(), CV_8UC3);

    cv::findContours(blobCopy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::cout << "Find " << contours.size() << " blobs. \n";

    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::RotatedRect rot_rect = cv::minAreaRect( cv::Mat(contours[i]) );
        cv::Point center = rot_rect.center;
        validateMarkers(depth_, center, -1);
    }

    color_.copyTo(colorMask, blobs);
    cv::addWeighted(colorMask, 0.8, color_, 0.2, 0, colorMask);
    cv::imshow("blobs", colorMask);

    result = marker_coordinates_.size() == iBlobs;

    return result;
}

bool MarkerBasedCalibration::validateMarkers(cv::Mat& depth, cv::Point center, int radius)
{
    bool result = false;

    cv::Point3f center3d;
    int NN = 3, NC = 0;
    int mx = center.x, my = center.y, mz = 0;
    
    for(int r=my-NN; r<my+NN; r++)
    {
        const cv::Point3f* cloud_ptr = (const cv::Point3f*)cloud_.ptr(r);
        for(int c=mx-NN; c<mx+NN; c++)        {

            if (cloud_ptr[c].z > 0)
            {
                center3d.x += cloud_ptr[c].x;
                center3d.y += cloud_ptr[c].y;
                center3d.z += cloud_ptr[c].z;
                
                NC++;
            } 
        }
    }
    center3d.x /= NC;
    center3d.y /= NC;
    center3d.z /= NC;

    mz = (int)(center3d.z * 1000);

    center3d.z = -center3d.z;

    if (radius < 0) 
    {
        marker_coordinates_.push_back(center3d);
        result = true;
    }
    else
    {
        // Get real radius of the circle
        int cl = mx - radius, cr = mx + radius;
        float left = cloud_.at<cv::Point3f>(my, cl).x;
        float right = cloud_.at<cv::Point3f>(my, cr).x;
        float real_radius = std::abs(left-right);
        float DELTA = 0.1;
        if (std::abs(real_radius/MARKER_RADIUS_ - 1) < DELTA) 
        {
            marker_coordinates_.push_back(center3d);
            result = true;
        }
    }

    return result;
}

bool compareMarkerPos ( cv::Point3f p1, cv::Point3f p2 ) {
    /* Sort the markers in Top-Down, Left-Right order */
    int ox = -500, oy = 500;
    int p1x = (int)(p1.x * 100), p1y = (int)(p1.y * 100);
    int p2x = (int)(p2.x * 100), p2y = (int)(p2.y * 100);
    int v1 = 10 * std::abs(p1y - oy) + (p1x - ox);
    int v2 = 10 * std::abs(p2y - oy) + (p2x - ox);
    return ( v1 < v2 );
}

bool MarkerBasedCalibration::estimateShelfDimensions()
{
    bool result = false;

    std::sort(marker_coordinates_.begin(), marker_coordinates_.end(), compareMarkerPos);

    //////////////////////////////////////////////////////////////////////////
    /// Estimate front bar
    CloudT::Ptr markers (new CloudT());
    for (size_t k=0; k<MARKER_COUNT_; k++) 
    {
        PointT pt (marker_coordinates_[k].x, marker_coordinates_[k].y, marker_coordinates_[k].z);
        markers->points.push_back(pt);
    }

    pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud(markers);
    seg.segment(*inliers, *plane_coeff);

    if (inliers->indices.size() > 0) 
    {
        for (int i=0; i<4; i++) 
        {
            front_bar_coeff_[i] = plane_coeff->values[i];
        }
        found_front_bar_ = true;
        printf(" Front bar coeff: %f, %f, %f, %f\n", 
            front_bar_coeff_[0], front_bar_coeff_[1], front_bar_coeff_[2], front_bar_coeff_[3]); 
    }

    //////////////////////////////////////////////////////////////////////////
    /// Estimate back wall
    CloudT::Ptr cloud_xyz (new CloudT()), cloud_bw (new CloudT()), cloud_hp (new CloudT());
    pcl::copyPointCloud(*pcl_cloud_, *cloud_xyz);
    const float leaf = 0.01; // Unit: meter
    
    float minZ = marker_coordinates_[0].z - DEFAULT_BIN_DEPTH_MM * 0.001 - 0.1, maxZ = minZ + 0.2;
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud_xyz);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (minZ, maxZ);
    pass.filter (*cloud_bw);

    inliers->indices.clear();
    plane_coeff->values.clear();
    seg.setInputCloud(cloud_bw);
    seg.segment(*inliers, *plane_coeff);

    if (inliers->indices.size() > 0) 
    {
        for (int i=0; i<4; i++) 
        {
            back_wall_coeff_[i] = plane_coeff->values[i];
        }
        found_back_wall_ = true;
        printf(" Back wall coeff: %f, %f, %f, %f\n", 
            back_wall_coeff_[0], back_wall_coeff_[1], back_wall_coeff_[2], back_wall_coeff_[3]); 
    }

    //////////////////////////////////////////////////////////////////////////
    /// Estimate horizontal plane
    /*inliers->indices.clear();
    plane_coeff->values.clear();
    cloud_bw->points.clear();
    minZ = marker_coordinates_[0].z - DEFAULT_BIN_DEPTH_MM * 0.001 + 0.1, maxZ = minZ + 0.1; 
    float minY = marker_coordinates_[3].y - 0.1, maxY = minY + 0.1;
    Eigen::Vector4f horiz_plane_coeff;
    pass.setInputCloud (cloud_xyz);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (minY, maxY);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (minZ, maxZ);
    pass.filter (*cloud_bw);

    seg.setInputCloud(cloud_bw);
    seg.segment(*inliers, *plane_coeff);

    if (inliers->indices.size() > 0) 
    {
        for (int i=0; i<4; i++) 
        {
            horiz_plane_coeff[i] = plane_coeff->values[i];
        }
        printf(" Bin bottom plane coeff: %f, %f, %f, %f\n", 
            horiz_plane_coeff[0], horiz_plane_coeff[1], horiz_plane_coeff[2], horiz_plane_coeff[3]); 
    }*/

    //////////////////////////////////////////////////////////////////////////
    /// Estimate kinect orientation
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q;
    Eigen::Vector3f rot_axis;
    Eigen::Vector3f orie_ref_p(0,0,1), orie_ref_n(0,0,-1); // Z axis
    Eigen::Vector3f orie_plane = front_bar_coeff_.head<3>();
    float theta, a0, a1;

    a0 = std::acos( orie_plane.dot(orie_ref_p) );
    a1 = std::acos( orie_plane.dot(orie_ref_n) );
    theta = std::abs(a0) < std::abs(a1) ? a0 : a1;
    //theta = a0;
    rot_axis = (orie_ref_p.cross(orie_plane)).normalized();
    q = Eigen::AngleAxisf( -theta, rot_axis );
    t = q.normalized();

    //orie_ref_p = Eigen::Vector3f::UnitY();
    //orie_ref_n = -orie_ref_p;
    //orie_plane = horiz_plane_coeff.head<3>();
    //a0 = std::acos( orie_plane.dot(orie_ref_p) );
    //a1 = std::acos( orie_plane.dot(orie_ref_n) );
    //theta = std::abs(a0) < std::abs(a1) ? a0 : a1;
    //rot_axis = (orie_ref_p.cross(orie_plane)).normalized();
    //q = Eigen::AngleAxisf( -theta, rot_axis );
    //t *= q.normalized();

    pcl::getEulerAngles(t, kinect_angle_[0], kinect_angle_[1], kinect_angle_[2]);
    kinect_orient_ = t;

    printf(" Kinect angle: %f, %f, %f\n", RAD2DEG(kinect_angle_[0]), RAD2DEG(kinect_angle_[1]), RAD2DEG(kinect_angle_[2])); 

    //////////////////////////////////////////////////////////////////////////
    /// Estimate bin depth
    bin_depth_ = 0;
    float MARKER_FRONTBAR_OFFSET = 0.02;
    for (size_t k=0; k<MARKER_COUNT_; k++) 
    {
        bin_depth_ += pcl::pointToPlaneDistance(markers->points[k], back_wall_coeff_) - MARKER_FRONTBAR_OFFSET;
    }
    bin_depth_ /= MARKER_COUNT_;

    //////////////////////////////////////////////////////////////////////////
    // Estimate width and height of each bin
    if (found_back_wall_) 
    {
        CloudT::Ptr markers_align (new CloudT());
        pcl::transformPointCloud(*markers, *markers_align, kinect_orient_);
        
        float THRESH = 0.03;
        std::vector<float> bin_wall_x, bin_wall_y;

        bin_wall_x.push_back(markers_align->points[0].x);
        bin_wall_y.push_back(markers_align->points[0].y);

        for (size_t i=1; i<markers_align->points.size(); i++) 
        {
            float xval = markers_align->points[i].x;
            float yval = markers_align->points[i].y;

            bool newX = true;
            for (size_t j=0; j<bin_wall_x.size(); j++ ) 
            {
                float xdiff = std::abs(bin_wall_x[j] - xval);
                if (xdiff < THRESH) 
                {
                    bin_wall_x[j] += xval;
                    bin_wall_x[j] /= 2;
                    newX = false;
                    break;
                } 
            }
            if (newX) 
            {
                bin_wall_x.push_back(xval);
            }

            bool newY = true;
            for (size_t j=0; j<bin_wall_y.size(); j++ ) 
            {
                float ydiff = std::abs(bin_wall_y[j] - yval);
                if (ydiff < THRESH) 
                {
                    bin_wall_y[j] += yval;
                    bin_wall_y[j] /= 2;
                    newY = false;
                    break;
                } 
            }
            if (newY) 
            {
                bin_wall_y.push_back(yval);
            }
        }
    
        std::sort(bin_wall_x.begin(), bin_wall_x.end(), std::less<int>());
        std::sort(bin_wall_y.begin(), bin_wall_y.end(), std::greater<int>());
        printf("Found %i horizonal walls and %i vertical walls\n", bin_wall_x.size(), bin_wall_y.size());

        if ( bin_wall_x.size() == SIZE_VERT_WALL_ && bin_wall_y.size() == SIZE_HORI_WALL_ ) 
        {
            Eigen::Affine3f tf = kinect_orient_.inverse();
            CloudT bar_centers_ori;
            int BIN_COUNT = (SIZE_VERT_WALL_-1)*(SIZE_HORI_WALL_-1);
            bar_centers_ori.points.resize(BIN_COUNT);
            allBinSize_ = cv::Mat::zeros(BIN_COUNT, 3, CV_32FC1);

            for (int ii=1; ii<SIZE_HORI_WALL_; ii++) 
            {
                float bin_height = bin_wall_y[ii-1] - bin_wall_y[ii];

                for (int jj=1; jj<SIZE_VERT_WALL_; jj++) 
                {
                    float bin_width = bin_wall_x[jj] - bin_wall_x[jj-1];
                    int r = (ii-1)*(SIZE_VERT_WALL_-1) + (jj-1);

                    allBinSize_.at<float>(r, 0) = bin_width;
                    allBinSize_.at<float>(r, 1) = bin_height;
                    allBinSize_.at<float>(r, 2) = bin_depth_;


                    PointT cp;
                    cp.x = bin_wall_x[jj-1] + bin_width*0.5;
                    cp.y = bin_wall_y[ii];
                    cp.z = markers_align->points[0].z;
                    bar_centers_ori.points[r] = cp;
                }
            }
            pcl::transformPointCloud(bar_centers_ori, bar_centers_, tf);

            CloudT grid_points_ori;
            int N = SIZE_VERT_WALL_*SIZE_HORI_WALL_;
            int count = 0;
            grid_points_ori.points.resize(N*2);

            for (int ii=0; ii<SIZE_HORI_WALL_; ii++) 
            {
                for (int jj=0; jj<SIZE_VERT_WALL_; jj++) 
                {
                    PointT pt;
                    pt.x = bin_wall_x[jj];
                    pt.y = bin_wall_y[ii];
                    pt.z = markers_align->points[0].z - MARKER_FRONTBAR_OFFSET;

                    grid_points_ori.points[count] = pt;

                    pt.z -= bin_depth_;
                    grid_points_ori.points[count + N] = pt;

                    count++;
                }
            }
            pcl::transformPointCloud(grid_points_ori, grid_points_, tf);

            /*
            /// Estimate the grid order of the markers
            int map_count = 0;
            for (size_t i=0; i<marker_coordinates_.size(); i++) 
            {
                float xval = marker_coordinates_[i].x;
                float yval = marker_coordinates_[i].y;
                int id_x = -1, id_y = -1;

                for (size_t j=0; j<xmean.size(); j++ ) 
                {
                    float xdiff = std::abs(xmean[j] - xval);
                    if (xdiff < THRESH) 
                    {
                        id_x = j;
                        break;
                    } 
                }

                for (size_t j=0; j<ymean.size(); j++ ) 
                {
                    float ydiff = std::abs(ymean[j] - yval);
                    if (ydiff < THRESH) 
                    {
                        id_y = j;
                        break;
                    } 
                }

                if (id_x >= 0 && id_y >= 0) 
                {
                    markers_on_shelf_.at<cv::Point3f>(id_y * SIZE_VERT_WALL + id_x, 0) = marker_coordinates_[i];
                    map_count++;
                }
            }

            if (map_count == MARKER_COUNT_) 
            {
                result = true;
                std::cout << "Map markers to shelf SUCCEEDED !!!\n";
            }
            */
        }             

    }

    return result;
}

bool MarkerBasedCalibration::loadParameters(std::string filename)
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

        fs["Number of Markers"] >> MARKER_COUNT_;
        fs["Marker Radius"] >> MARKER_RADIUS_;

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
}

bool MarkerBasedCalibration::saveParameters(std::string filename)
{
    bool result = false;

    std::cout << "Saving parameters to " << filename << std::endl;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        // write current date and time
        time_t rawtime; 
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));

        fs << "Hue Threshold" << "[" << iLowH << iHighH << "]";
        fs << "Saturation Threshold" << "[" << iLowS << iHighS << "]";
        fs << "Value Threshold" << "[" << iLowV << iHighV << "]";
        fs << "Depth Threshold" << "[" << iNearD << iFarD << "]";

        fs << "Number of Markers" << MARKER_COUNT_;
        fs << "Marker Radius" << MARKER_RADIUS_;

        result = true;
    }

    return result;
}

bool MarkerBasedCalibration::saveResults(std::string filename)
{
    bool result = false;

    std::cout << "Saving calibration results to " << filename << std::endl;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        // write current date and time
        time_t rawtime; 
        time(&rawtime);
        fs << "Calibration Date" << asctime(localtime(&rawtime));

        cv::Mat bc = cv::Mat::zeros(bar_centers_.points.size(), 3, CV_32FC1);
        for (size_t i=0;i<bar_centers_.points.size();i++) 
        {
            bc.at<float>(i,0) = bar_centers_.points[i].x;
            bc.at<float>(i,1) = bar_centers_.points[i].y;
            bc.at<float>(i,2) = bar_centers_.points[i].z;
        }

        fs << "Bar Centers" << bc;
        fs << "Marker Position" << cv::Mat(marker_coordinates_).reshape(1, MARKER_COUNT_);
        fs << "Bin Size" << allBinSize_;
        fs << "Kinect Angle" << cv::Mat(kinect_angle_).t();

        result = true;
    }

    return result;
}

bool MarkerBasedCalibration::copyParamFile(std::string src, std::string des)
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

void MarkerBasedCalibration::copyCloud(cv::Mat& color, cv::Mat& cvCloud, ColorCloudT::Ptr pclCloud)
{
    pclCloud->clear();

    for(int r=0; r<cvCloud.rows; r++)
    {
        cv::Vec3b* color_ptr = (cv::Vec3b*)color.ptr(r);
        cv::Point3f* cloud_ptr = (cv::Point3f*)cvCloud.ptr(r);
        for(int c=0; c<cvCloud.cols; c++)
        {
            if (cloud_ptr[c].z > 0)
            {
                ColorPointT pt;
                pt.x = cloud_ptr[c].x;
                pt.y = cloud_ptr[c].y;
                pt.z = -cloud_ptr[c].z;
                pt.r = color_ptr[c][2];
                pt.g = color_ptr[c][1];
                pt.b = color_ptr[c][0];

                pclCloud->points.push_back(pt);
            } 
        }
    }
}


void MarkerBasedCalibration::showResults()
{
    cv::imshow("color", color_);
    viewer_.removeAllPointClouds();
    viewer_.addPointCloud(pcl_cloud_);

    if (new_frame_)
    {
        hsvSpectrum();

        viewer_.removeCoordinateSystem();
        viewer_.removeAllShapes();
        Eigen::Affine3f world_frame = kinect_orient_.inverse();
        viewer_.addCoordinateSystem(0.15, world_frame);
        // camera model
        {
            for (int i=0; i<4; i++) 
            {
                PointT pt0 = camera_model_.points[0];
                PointT pt1 = camera_model_.points[i+1];
                PointT pt2 = camera_model_.points[(i+1)%4+1];
                std::stringstream ss;

                ss << "cam_line_" << i;
                viewer_.addLine(pt0, pt1, 1,0.6,0, ss.str());
                ss << i;
                viewer_.addLine(pt2, pt1, 1,0.6,0, ss.str());
            }
        }
        
        if (found_markers_) 
        {
            for (size_t i = 0; i < marker_coordinates_.size(); i++) 
            {
                PointT center (marker_coordinates_[i].x, marker_coordinates_[i].y, marker_coordinates_[i].z);
                double radius = 0.02;
                std::stringstream ss;
                ss << "marker_" << i;
                viewer_.addSphere(center, radius, 0,0,1, ss.str());
            }


            for (size_t i = 0; i < bar_centers_.size(); i++) 
            {
                PointT center (bar_centers_[i].x, bar_centers_[i].y, bar_centers_[i].z);
                double radius = 0.01;
                std::stringstream ss;
                ss << "bar_centers_" << i;
                viewer_.addSphere(center, radius, 1,0,1, ss.str());
            }
        }

        int N = SIZE_VERT_WALL_*SIZE_HORI_WALL_;
        if (grid_points_.size() == N*2) 
        {
            for (int i=0; i<SIZE_HORI_WALL_; i++) 
            {
                PointT pt1 = grid_points_.points[i*SIZE_VERT_WALL_];
                PointT pt2 = grid_points_.points[i*SIZE_VERT_WALL_ + SIZE_VERT_WALL_ -1];

                std::stringstream ss;
                ss << "hori_" << i;
                viewer_.addLine(pt1, pt2, 1, 0, 0, ss.str());


                pt1 = grid_points_.points[i*SIZE_VERT_WALL_ + N];
                pt2 = grid_points_.points[i*SIZE_VERT_WALL_ + SIZE_VERT_WALL_ -1+ N];

                ss.clear();
                ss << "bw_hori_" << i;
                viewer_.addLine(pt1, pt2, 0.7, 0, 0, ss.str());
            }

            int BT = (SIZE_HORI_WALL_ - 1) * SIZE_VERT_WALL_;
            for (int i=0; i<SIZE_VERT_WALL_; i++) 
            {
                PointT pt1 = grid_points_.points[i];
                PointT pt2 = grid_points_.points[i + BT];

                std::stringstream ss;
                ss << "vert_" << i;
                viewer_.addLine(pt1, pt2, 0, 1, 0, ss.str());

                pt1 = grid_points_.points[i+ N];
                pt2 = grid_points_.points[i + BT+ N];

                ss.clear();
                ss << "bw_vert_" << i;
                viewer_.addLine(pt1, pt2, 0, 0.7, 0, ss.str());
            }

            for (int i=0; i<N; i++) 
            {
                PointT pt1 = grid_points_.points[i];
                PointT pt2 = grid_points_.points[i + N];

                std::stringstream ss;
                ss << "edge_" << i;
                viewer_.addLine(pt1, pt2, 0, 0, 0.7, ss.str());
            }
        }

        //if (found_back_wall_) 
        //{
        //    pcl::ModelCoefficients plane_coeff;
        //    for (int i=0; i<4; i++)
        //        plane_coeff.values.push_back( back_wall_coeff_[i] );
        //    viewer_.addPlane(plane_coeff, "back_wall");
        //}

        //if (found_front_bar_) 
        //{
        //    pcl::ModelCoefficients plane_coeff;
        //    for (int i=0; i<4; i++)
        //        plane_coeff.values.push_back( front_bar_coeff_[i] );
        //    viewer_.addPlane(plane_coeff, "front_bar");
        //}

        /*viewer_.setCameraPosition(
            -1.71525, 0.00405309, 2.14761,
            0.0528733, -0.180152, -1.11601,
            0.1315, 0.991198, 0.0152971);*/
    }


    viewer_.spinOnce();
}
