#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
std::string rgbd_data, model_name, bin_name;
cv::Rect roiRect;

bool bDevelopmentMode_ = true;
bool drawing_box = false;

void onMouse(int event, int x, int y, int flag, void* param)
{
    switch( event ){
    case CV_EVENT_MOUSEMOVE: 
        if( drawing_box ){
            roiRect.width = x-roiRect.x; if (roiRect.width < 10) roiRect.width = 10;
            roiRect.height = y-roiRect.y; if (roiRect.height < 10) roiRect.height = 10;
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
        fs["Bin Name"] >> bin_name;

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
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


int main (int argc, char* argv[])
{
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

    if (bDevelopmentMode_) {
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);
        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);
        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
        cvCreateTrackbar("NearD", "Control", &iNearD, 1000);
        cvCreateTrackbar("FarD", "Control", &iFarD, 1000);
        cv::setMouseCallback("image", onMouse);
    }


    PoseEstimator poseEstimator;
    PointCloudT::Ptr model (new PointCloudT);
    int executeAlign = 0, blob_index = 0, key;
    ImageMatching matcher;
    cv::Mat imRoi, des;
    std::vector<cv::KeyPoint> kps;
    std::vector<std::vector<cv::KeyPoint> > obj_kps_set(6);
    std::vector<cv::Mat> obj_des_set(6), obj_img_set(6);


    for (;;) 
    {
        cv::Mat color, depth, mask, maskedImage, blob, cloud, sub_cloud;
        PointCloudT::Ptr sample (new PointCloudT);
        PointCloudT::Ptr modle_aligned (new PointCloudT);

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

        if (!depth.empty() && !color.empty())
        {
            capture.generateMask(color, depth, mask, 3);
            color.copyTo(maskedImage, mask);
            poseEstimator.extractBlob(mask, blob_index, blob);

            cv::imshow("image", color);
            cv::imshow("mask", mask);
            if (blob.empty()) 
            {
                std::cout << "Blob map is EMPTY!!\n";
            } 
            else 
            {
                cv::imshow("blob", blob);
                std::vector<std::vector<cv::Point> > contours;
                cv::findContours(blob.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                roiRect = cv::boundingRect(contours[0]);
            }

            kps.clear();
            
            printf("Press 'a' to execute alignment; 'b' to change blob index; 'q' to quit and proceed next stage.\n");
            key = cv::waitKey() % 255;   

            while (key != 'q')
            {   
                if (key == 'b')
                {
                    blob_index++;
                    poseEstimator.extractBlob(mask, blob_index, blob);

                    if (blob.empty()) 
                    {
                        std::cout << "Blob map is EMPTY!!\n";
                    } 
                    else 
                    {
                        cv::imshow("blob", blob);
                        std::vector<std::vector<cv::Point> > contours;
                        cv::findContours(blob.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                        roiRect = cv::boundingRect(contours[0]);
                    }
                }
                else if (key == 'a')
                {
                    imRoi = color(roiRect);
                    matcher.extractFeatures(imRoi, kps, des);
                }

                cv::Mat color_show = color.clone();
                if (kps.size() > 0) 
                {
                    cv::Mat colorRoi = color_show(roiRect);
                    cv::drawKeypoints(colorRoi, kps, colorRoi, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                }
                cv::rectangle(color_show, roiRect, CV_RGB(255,0,0));
                cv::imshow("image", color_show);

                key = cv::waitKey(30) % 255;
            } // End >> while (key != 'q')            
        }

        //if (kps.size() > 0) 
        {
            int N = obj_kps_set.size();
            int id = fid % N;
            obj_kps_set[id].swap(kps);
            obj_des_set[id] = des.clone();
            obj_img_set[id] = color(roiRect).clone();
            std::string faces[6] = {"front", "right", "back", "left", "up", "down"};

            if (id == (N-1)) 
            {
                std::string lib_file = cv::format(_LIB3D_DIRECTORY_"/obj_%i.yml", (fid-(N-1))/N);
                cv::FileStorage fs(lib_file.c_str(), cv::FileStorage::WRITE);
                if (fs.isOpened()) 
                {
                    for (size_t i=0; i<obj_kps_set.size(); i++) 
                    {
                        fs << faces[i]+"_keypoint" << obj_kps_set[i];
                        fs << faces[i]+"_descriptor" << obj_des_set[i];
                        fs << faces[i]+"_img" << obj_img_set[i];
                    }
                    fs.release();
                }
                std::cout << "Saved lib2d file to " << lib_file << std::endl;
            }
        }

        key = cv::waitKey() % 255;
        if (key == 27 || key == 'q')
            break;
#ifdef _USE_VIRTUAL_KINECT_
        else if (key == 'f')
        {
            fid++;
        }
        else if (key == 'v')
        {
            fid--;
            if (fid<0) fid = 0;
        }
#endif
    }


    return 0;
}

