#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "RGBD_Capture.h"
#include "VirtualKinect.h"

#define _USE_VIRTUAL_KINECT_

int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 170;
int iHighV = 255;

int iNearD = 400;
int iFarD = 800;

int fid = 0, image_count = 0;
int nMultiGrab = 4;
std::string rgbd_data, bin_name, fid2bin;
cv::Rect roiRect;

float cameraPitchAngle = 11.0f;
bool bDevelopmentMode_ = true;
bool drawing_box = false, flip_image_ = false, save_image = false;

void onMouse(int event, int x, int y, int flag, void* param);
bool loadParameters(std::string filename);
bool saveParameters(std::string filename);


int main (int argc, char* argv[])
{
    loadParameters(_PARAM_DIRECTORY_"/../../../registration/params/param_test.yml");

#ifdef _USE_VIRTUAL_KINECT_
    VirtualKinectCapture kinect(rgbd_data);
#else
    cv::VideoCapture kinect(CV_CAP_OPENNI);
#endif // _USE_VIRTUAL_KINECT_

    RGBDCapture capture;
    capture.setBinName(bin_name);

    if (!kinect.isOpened()) {
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

    for (;;) 
    {
        cv::Mat color, depth, mask, maskedImage, points;

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
            capture.generateMask(color, depth, mask, 3);

            if (save_image) 
            {
                std::stringstream ss;
                ss << "rgbImage_" << image_count << ".png";
                imwrite(ss.str(), color);
                ss.str("");
                ss << "depthImage_" << image_count << ".png";
                imwrite(ss.str(), depth);
                ss.str("");
                ss << "maskImage_" << image_count << ".png";
                imwrite(ss.str(), mask);
                image_count++;
                save_image = false;
            }

            roiRect = capture.getROI();
            cv::rectangle(color, roiRect, CV_RGB(255,0,0));
            cv::imshow("image", color);

            color.copyTo(maskedImage, mask);
            cv::imshow("mask", maskedImage);
            //cv::imshow("mask", mask);

        }

        int key = cv::waitKey() % 255;
        if (key == 27 || key == 'q')
            break;
        //else if (key == 's')
        //{
        //    if (bDevelopmentMode_)
        //        saveParameters(_PARAM_DIRECTORY_"/parameters/param.yml");
        //}
#ifdef _USE_VIRTUAL_KINECT_
        else if (key == 'f')
        {
            fid++;
            if (!fid2bin.empty()) 
            {
                fid = fid % fid2bin.length();
                bin_name = fid2bin[ fid ];
                bin_name = "bin_" + bin_name;
            }
            capture.setBinName(bin_name);
        }
        else if (key == 'v')
        {
            fid--;
            if (!fid2bin.empty()) 
            {
                fid = fid % fid2bin.length();
                bin_name = fid2bin[ fid ];
                bin_name = "bin_" + bin_name;
            }
            capture.setBinName(bin_name);
        }
#endif
        else if (key == 's')
        {
            save_image = true;            
        }
    }


    return 0;
}


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

    std::cout << "Loading parameters from " << filename << std::endl;

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

        fs["Camera Pitch Anlge"] >> cameraPitchAngle;
        fs["FrameID"] >> fid; 
        fs["RGBD File"] >> rgbd_data;
        fs["Bin Name"] >> bin_name;
        fs["Multi-Grab"] >> nMultiGrab;
        fs["Fid2Bin"] >> fid2bin;

#ifdef _USE_VIRTUAL_KINECT_
        if (fid2bin.length() > 0) 
        {
            fid = fid % fid2bin.length();
            bin_name = fid2bin[ fid ];
            bin_name = "bin_" + bin_name;
        }
#endif // _USE_VIRTUAL_KINECT_

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
}

bool saveParameters(std::string filename)
{
    bool result = false;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        // write current date and time
        time_t rawtime; 
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));

        fs << "isDevelopmentMode" << bDevelopmentMode_;
        fs << "Hue Threshold" << "[" << iLowH << iHighH << "]";
        fs << "Saturation Threshold" << "[" << iLowS << iHighS << "]";
        fs << "Value Threshold" << "[" << iLowV << iHighV << "]";
        fs << "Depth Threshold" << "[" << iNearD << iFarD << "]";
        fs << "ROI region" << roiRect;
        fs << "Camera Pitch Anlge" << cameraPitchAngle;
        fs << "FrameID" << fid;
        fs << "RGBD File" << rgbd_data;
        
        result = true;
    }

    return result;
}

