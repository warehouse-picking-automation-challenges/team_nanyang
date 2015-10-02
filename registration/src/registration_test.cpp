#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>

#include "registration.h"
#include "../../capture/capture_kinect/include/RGBD_Capture.h"
#include "../../capture/capture_kinect/include/VirtualKinect.h"

#define _USE_VIRTUAL_KINECT_

int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 170;
int iHighV = 255;

int iNearD = 400;
int iFarD = 800;

int fid = 0;
int nMultiGrab = 4;
std::string rgbd_data, model_name, bin_name, fid2bin;
cv::Rect roiRect;

bool bDevelopmentMode_ = true;
bool drawing_box = false;
bool use_sample_model_ = false, flip_image_ = false;

void onMouse(int event, int x, int y, int flag, void* param)
{
    switch( event ){
    case CV_EVENT_MOUSEMOVE: 
        if( drawing_box ){
            roiRect.width = x-roiRect.x;
            roiRect.height = y-roiRect.y;
        }
        break;

    case CV_EVENT_LBUTTONDOWN:
        drawing_box = true;
        roiRect = cvRect( x, y, 10, 10 );
        break;

    case CV_EVENT_LBUTTONUP:
        drawing_box = false;
        if( roiRect.width < 0 ){
            roiRect.x += roiRect.width;
            roiRect.width *= -1;
        }
        if( roiRect.height < 0 ){
            roiRect.y += roiRect.height;
            roiRect.height *= -1;
        }
        break;
    }
}

bool loadParameters(std::string filename)
{
    bool result = false;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["isDevelopmentMode"] >> bDevelopmentMode_;
        fs["useSampleModel"] >> use_sample_model_;

        cv::FileNodeIterator fit  = fs["Hue Threshold"].begin();
        fit >> iLowH >> iHighH;
        fit  = fs["Saturation Threshold"].begin();
        fit >> iLowS >> iHighS;
        fit  = fs["Value Threshold"].begin();
        fit >> iLowV >> iHighV;
        fit  = fs["Depth Threshold"].begin();
        fit >> iNearD >> iFarD;

        fs["ROI region"] >> roiRect;
        fs["FrameID"] >> fid;
        fs["RGBD File"] >> rgbd_data;
        fs["Model Name"] >> model_name;
        fs["Bin Name"] >> bin_name;
        fs["Multi-Grab"] >> nMultiGrab;
        fs["Fid2Bin"] >> fid2bin;
        fs["Flip Image"] >> flip_image_;

        if (fid2bin.length() > 0) 
        {
            fid = fid % fid2bin.length();
            bin_name = fid2bin[ fid ];
            bin_name = "bin_" + bin_name;
        }

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
}

bool loadOBJdata(std::string filename, PointCloudT::Ptr cloud)
{
    // Load the OBJ file
    pcl::console::TicToc tt; tt.tic();
    pcl::console::print_highlight ("Loading "); pcl::console::print_value ("%s ", filename.c_str());
    
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(filename.c_str(), mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    std::cout << "[done,  " << tt.toc () << " ms : " << cloud->points.size() << " points]\n";

    return true;
}

void printHelp()
{
    std::cout << "Usage:\n";
#ifdef _USE_VIRTUAL_KINECT_
    std::cout << "\tregistration_test frame_id recorded_data_file_list path_of_3d_mesh_file\n";
#else
    std::cout << "\tregistration_test path_of_3d_mesh_file\n";
#endif // _USE_VIRTUAL_KINECT_
    std::cout << "Key press:\n";
#ifdef _USE_VIRTUAL_KINECT_
    std::cout << "\t'f'--Capture next frame (fid++)\n";
    std::cout << "\t'v'--Capture previous frame (fid++)\n";
#endif
    std::cout << "\t's'--Save current parameters\n";
    std::cout << "\t'b'--Change blob_index\n";
    std::cout << "\t'a'--Execute alignment\n";
}

#ifdef _IS_WINDOWS_

LONG WINAPI VectoredExceptionHandler(PEXCEPTION_POINTERS pExceptionInfo)
{
    std::cout << "Program exit. Code: " << std::hex << pExceptionInfo->ExceptionRecord->ExceptionCode << std::endl;

    //return EXCEPTION_CONTINUE_SEARCH;

    exit(0);
}

#endif // _IS_WINDOWS_

int main (int argc, char* argv[])
{
#ifdef _IS_WINDOWS_
    AddVectoredExceptionHandler(1, VectoredExceptionHandler);
#endif // _IS_WINDOWS_

    printHelp();
    loadParameters(_PARAM_DIRECTORY_"/param_test.yml");

#ifdef _USE_VIRTUAL_KINECT_
    std::cout << "RGBD File: " << rgbd_data << std::endl;
    VirtualKinectCapture kinect(rgbd_data);
#else
    cv::VideoCapture kinect(CV_CAP_OPENNI);
#endif // _USE_VIRTUAL_KINECT_

    RGBDCapture capture;
    capture.setBinName(bin_name);

    if (!kinect.isOpened()) 
    {
        std::cout << "Can not open a Kinect object.\n";
        return -1;
    }

    PoseEstimator poseEstimator;
    PointCloudT::Ptr model (new PointCloudT);
    int executeAlign = 0, blob_index = 0, key;

    if (!poseEstimator.loadModel( model_name, model ))
    {
        std::cout << "Can not load model file. \n";
        return -1;
    }
    poseEstimator.setBinName(bin_name);

    //if (bDevelopmentMode_) {
    //    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
    //    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    //    //Create trackbars in "Control" window
    //    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    //    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
    //    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    //    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
    //    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    //    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
    //    cvCreateTrackbar("NearD", "Control", &iNearD, 1000);
    //    cvCreateTrackbar("FarD", "Control", &iFarD, 1000);
    //    cv::setMouseCallback("image", onMouse);
    //}

    for (;;) 
    {
        cv::Mat color, depth, mask, maskedImage, blob, cloud, sub_cloud;
        PointCloudT::Ptr sample (new PointCloudT);
        PointCloudT::Ptr modle_aligned (new PointCloudT);

        if (nMultiGrab > 1) 
        {
            printf("Grabing %i Kinect frames start from ID #%i ", nMultiGrab, fid*nMultiGrab);

            std::vector<cv::Mat> multiDepths;
            do
            {
                cv::Mat depth;
#ifdef _USE_VIRTUAL_KINECT_
                if (kinect.grab(fid * nMultiGrab + multiDepths.size()))
#else
                if( kinect.grab() )
#endif // _USE_VIRTUAL_KINECT_                
                {
                    if (kinect.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP))
                        multiDepths.push_back(depth);

                    if (multiDepths.size() == nMultiGrab)
                    {
                        kinect.retrieve(color, CV_CAP_OPENNI_BGR_IMAGE);
                        std::cout << ". ";
                    }
                } 
            } 
            while (multiDepths.size() < nMultiGrab);
            std::cout << "Done.\n";

            capture.averagingDepthMaps(multiDepths, depth);
        }
        else
        {
#ifdef _USE_VIRTUAL_KINECT_
            if (!kinect.grab(fid))
#else
            if( !kinect.grab() )
#endif // _USE_VIRTUAL_KINECT_                
            {
                std::cout << "Can not grab images." << std::endl;
            }
            else
            {
                kinect.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP);
                kinect.retrieve(color, CV_CAP_OPENNI_BGR_IMAGE);
            }
        }

        if (flip_image_) 
        {
            cv::flip(color, color, -1);
            cv::flip(depth, depth, -1); 
        } 

        if (!depth.empty() && !color.empty())
        {
            roiRect = cv::Rect(color.cols/4, color.rows/4, color.cols/2, color.rows/2);
            cv::rectangle(color, roiRect, CV_RGB(255,0,0));
            cv::imshow("image", color);
          
            capture.generateMask(color, depth, mask, 3);
            color.copyTo(maskedImage, mask);
            poseEstimator.extractBlob(mask, blob_index, blob);

            cv::imshow("mask", mask);
            if (!blob.empty()) cv::imshow("blob", blob);

            printf("Press 'a' to execute alignment; 'b' to change blob index; 'q' to quit and proceed next stage.\n");
            key = cv::waitKey() % 255;   

            while (key != 'q')
            {   
                if (key == 'b')
                {
                    blob_index++;
                    poseEstimator.extractBlob(mask, blob_index, blob);
                    if (!blob.empty()) cv::imshow("blob", blob);
                }
                else if (key == 'a')
                {
                    VirtualKinectCapture vk;
                    vk.generatePointClouds(depth, cloud);
                                        
                    sub_cloud.setTo(cv::Scalar::all(0));
                    cloud.copyTo(sub_cloud, blob.empty() ? mask : blob);

                    sample->clear();
                    poseEstimator.copyCloud(sub_cloud, sample);

                    if (!blob.empty()) 
                    {
                        std::vector<std::vector<cv::Point> > contours;
                        cv::findContours(blob.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                        roiRect = cv::boundingRect(contours[0]);
                        poseEstimator.setBlobImage( color(roiRect) );
                    }

                    //if (bDevelopmentMode_) 
                    //{
                    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgrCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                    // copyCloud(color, blob, cloud, bgrCloud);
                    // pcl::visualization::PCLVisualizer vis("BgrCloud");
                    // vis.addPointCloud(bgrCloud, "bgrCloud");
                    // vis.addCoordinateSystem(0.1);
                    // vis.spin();
                    //}

                    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), tmp_transformation = Eigen::Matrix4f::Identity();
                    float confidence;
                    std::vector<float> pose;

                    if (use_sample_model_)
                    {
                        pcl::io::loadPCDFile<PointNT> ("H:/chef.pcd", *model);
                        pcl::io::loadPCDFile<PointNT> ("H:/rs1.pcd", *sample);
                    }

                    //Eigen::Vector4f centroid;
                    //pcl::compute3DCentroid(*sample, centroid);              
                    //for (int ii=0; ii<3; ii++)
                    //    tmp_transformation(ii,3) = centroid[ii];
                    //pcl::transformPointCloud(*model, *model, tmp_transformation);

                    confidence = poseEstimator.getPose(model, sample, modle_aligned, transformation);
                    poseEstimator.transformationToPose(transformation, pose);
              
                    printf("Estimated pose: \n %.3f %.3f %.3f \n %.3f %.3f %.3f \n %.3f %.3f %.3f\n",
                        pose[0], pose[1], pose[2],
                        pose[3], pose[4], pose[5],
                        pose[6], pose[7], pose[8]);

                    if (bDevelopmentMode_) 
                    {
                        if (confidence > 0) 
                            std::cout << "Succeed to estimate object pose!\nConfidence: " << confidence << std::endl;
                        else 
                            std::cout << "Failed to estimate object pose!\n";
                    }

                }

                key = cv::waitKey(30) % 255;
            }             
        }

        key = cv::waitKey() % 255;
        if (key == 27 || key == 'q')
            break;
#ifdef _USE_VIRTUAL_KINECT_
        else if (key == 'f')
        {
            fid++;
            fid = fid % fid2bin.length();
            bin_name = fid2bin[ fid ];
            bin_name = "bin_" + bin_name;
            capture.setBinName(bin_name);
            poseEstimator.setBinName(bin_name);
        }
        else if (key == 'v')
        {
            fid--;
            fid = fid % fid2bin.length();
            bin_name = fid2bin[ fid ];
            bin_name = "bin_" + bin_name;
            capture.setBinName(bin_name);
            poseEstimator.setBinName(bin_name);
        }
#endif
    }


    return 0;
}

