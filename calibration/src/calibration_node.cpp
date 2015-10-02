

#include "calibration.h"
#include "../../capture/capture_kinect/include/VirtualKinect.h"

#ifdef _IS_WINDOWS_
#define _USE_VIRTUAL_KINECT_
#endif // _IS_LINUX_


int fid = 0, nMultiGrab = 1;
bool process_frame = true, caps_lock = false, shift_press = false;
std::string rgbd_data;

bool loadParameters(std::string filename);


int main (int argc, char* argv[])
{
#ifdef _USE_VIRTUAL_KINECT_
    loadParameters(_PARAM_DIRECTORY_"/../../registration/params/param_test.yml");

    VirtualKinectCapture kinect(rgbd_data);
#else
    cv::VideoCapture kinect(CV_CAP_OPENNI);
#endif // _USE_VIRTUAL_KINECT_

    VirtualKinectWriter vk;
    MarkerBasedCalibration calib;

    if (!kinect.isOpened()) {
        std::cout << "Can not open a Kinect object.\n";
        return -1;
    }

    printf("Grabing %i Kinect frames start from ID #%i ", nMultiGrab, fid*nMultiGrab);

    for (;;) 
    {
        cv::Mat color, depth, mask, maskedImage, points;

        if (nMultiGrab > 1) 
        {
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
                    }
                } 
            } 
            while (multiDepths.size() < nMultiGrab);

            vk.averagingDepthMaps(multiDepths, depth);
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

        if (!depth.empty() && !color.empty())
        {
            vk.generatePointClouds(depth, points);                
            calib.processFrame(color, depth, points, process_frame);
            process_frame = false;
        }

        char key = cv::waitKey(30) % 255;
        if (key == 27 || key == 'q')
            break;
        else if (key == 20) 
        {
            caps_lock ^= 1;
        }
        else if (key == 16) 
        {
            shift_press = true;
        }

        if ( caps_lock || ('a' <= key && key <= 'l' && shift_press)) 
        {
            key -= 32;
            shift_press = false;
        }

        if (key > 0)
            printf("Key: %c %i %i\n", key, key, 'A');

        if ('A' <= key && key <= 'L') 
        {
            std::string bin_name = "bin_";
            bin_name += key;
            std::string rgb_file = _OUTPUT_DIRECTORY_"/back_ground_images/"+bin_name+"_rgb.png";
            std::string depth_file = _OUTPUT_DIRECTORY_"/back_ground_images/"+bin_name+"_depth.png";
            cv::imwrite(rgb_file, color);
            cv::imwrite(depth_file, depth);
            printf("Saved back ground data:\n %s\n %s\n", rgb_file.c_str(), depth_file.c_str());
        }
        else if (key == 'p')
        {
            process_frame = true;
        }
#ifdef _USE_VIRTUAL_KINECT_
        else if (key == 'f')
        {
            fid ++;     
            process_frame = true;       
            printf("Grabing %i Kinect frames start from ID #%i ", nMultiGrab, fid*nMultiGrab);

        }
        else if (key == 'v')
        {
            fid --;
            process_frame = true;
            printf("Grabing %i Kinect frames start from ID #%i ", nMultiGrab, fid*nMultiGrab);
        }
#endif
    }

    return 0;
}


bool loadParameters(std::string filename)
{
    bool result = false;

    std::cout << "Loading parameters from " << filename << std::endl;

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["FrameID"] >> fid; 
        fs["RGBD File"] >> rgbd_data;
        fs["Multi-Grab"] >> nMultiGrab;

        result = true;
    }
    else
        std:: cout << "Fail to load parameter. Use default value.\n";

    return result;
}


