#include <iostream>

#include <string>
#include <vector>
#include <dirent.h>

#include "VirtualKinect.h"
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

int listdir(const char *path, std::vector<std::string>& depths, std::vector<std::string>& rgbs);
int getImFiles(const char *path, std::vector<std::string>& names);

int main (int argc, char* argv[])
{
	using namespace openni;

	bool outputIsLibrary = true;
	if (argc > 1) {
		if (!strcmp(argv[1], "test_data")) {
			std::cout << "Capturing data for testing.." << std::endl;
			outputIsLibrary = false;
		} else std::cout << "Capturing data for library.." << std::endl;
	} else std::cout << "Capturing data for library.." << std::endl;

	CreateDirectory ("Output", NULL);
	int frameGap = 0;

	std::string outputAddress;

	if (outputIsLibrary) {
		CreateDirectory ("Output/image_library", NULL);
		if (argc > 1 && argv[2] != '\0') {
			//if(argv[2] != '\0'){
				char folder[256];
				sprintf(folder, "Output/image_library/%s", argv[2]);
				CreateDirectory (folder, NULL);
				sprintf(folder, "Output/image_library/%s/rgbd", argv[2]);
				outputAddress = folder;
				CreateDirectory (outputAddress.c_str(), NULL);
				sprintf(folder, "Output/image_library/%s/rgbd/rgb", argv[2]);
				CreateDirectory (folder, NULL);
				sprintf(folder, "Output/image_library/%s/rgbd/depth", argv[2]);
				CreateDirectory (folder, NULL);
			//}
		}else{
			CreateDirectory ("Output/image_library/item", NULL);
			outputAddress = "Output/image_library/item/rgbd";
			CreateDirectory (outputAddress.c_str(), NULL);
			CreateDirectory ("Output/image_library/item/rgbd/rgb", NULL);
			CreateDirectory ("Output/image_library/item/rgbd/depth", NULL);
		}
		frameGap = 75;
	} else {
		outputAddress = "Output/test_data/";
		CreateDirectory (outputAddress.c_str(), NULL);
		CreateDirectory ("Output/test_data/rgb", NULL);
		CreateDirectory ("Output/test_data/depth", NULL);
		frameGap = 200;
	}

	OpenNI::initialize();
    puts( "Kinect initialization..." );
    Device device;
    if ( device.open( openni::ANY_DEVICE ) != 0 )
    {
        puts( "Kinect not found !" ); 
        return -1;
    }
    puts( "Kinect opened" );
    VideoStream depthV, colorV;
    colorV.create( device, SENSOR_COLOR );
    colorV.start();
    puts( "Camera ok" );
    depthV.create( device, SENSOR_DEPTH );
    depthV.start();
    puts( "Depth sensor ok" );
    VideoMode paramvideo;
    paramvideo.setResolution( 640, 480 );
    paramvideo.setFps( 30 );
    paramvideo.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );
    depthV.setVideoMode( paramvideo );
    paramvideo.setPixelFormat( PIXEL_FORMAT_RGB888 );
    colorV.setVideoMode( paramvideo );

    // If the depth/color synchronisation is not necessary, start is faster :
    //device.setDepthColorSyncEnabled( false );

    // Otherwise, the streams can be synchronized with a reception in the order of our choice :
    device.setDepthColorSyncEnabled( true );
    device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

    VideoStream** stream = new VideoStream*[2];
    stream[0] = &depthV;
    stream[1] = &colorV;
    puts( "Kinect initialization completed" );

	cv::Mat colorBuffer( cv::Size( 640, 480 ), CV_8UC3 );
    cv::Mat depthBuffer( cv::Size( 640, 480 ), CV_16UC1 ), depthTemp, depthDisplay;
	cv::namedWindow( "RGB", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Depth", CV_WINDOW_AUTOSIZE );
	bool firstDepth = true, firstColor = true;

	cv::Mat captureMat = cv::Mat::zeros(200, 200, CV_8UC3);

	int rgbFrameCount = 0, outputFrameCount = 0, lastRgbFrameCount = -1;
    for (;;) 
    {
        cv::Mat color, depth, mask, depthShow;
		
		if ( device.getSensorInfo( SENSOR_DEPTH ) != NULL ) {
			VideoFrameRef depthFrame, colorFrame;
			cv::Mat colorcv( cv::Size( 640, 480 ), CV_8UC3);
			cv::Mat depthcv( cv::Size( 640, 480 ), CV_16UC1);

			int changedIndex;
			if( device.isValid() ){
				OpenNI::waitForAnyStream( stream, 2, &changedIndex );
				switch ( changedIndex ){
					case 0:
						depthV.readFrame( &depthFrame );

						if ( depthFrame.isValid() ){
							if(firstDepth == true)	firstDepth = false;
							depthcv.data = (uchar*) depthFrame.getData();
						
							//To show depth image using entire range of viewed depth instead of default scale.
							//double min;
							//double max;
							//cv::minMaxIdx(depthcv, &min, &max);
							//cv::Mat adjMap = depthcv * 5;
							//cv::convertScaleAbs(depthcv, adjMap, 255 / max);
							//depthBuffer = adjMap.clone();
							cv::flip(depthcv, depthBuffer, 1);
							depthBuffer *= 5;
							depthBuffer.copyTo(depthTemp);
							depthTemp = 65535 - depthTemp;
							depthDisplay = cv::Mat::zeros(depthBuffer.size(), CV_16UC1);
							cv::Mat depthMask = depthBuffer > 0;
							depthTemp.copyTo(depthDisplay, depthMask);
							cv::imshow( "Depth", depthDisplay);
						}
						break;

					case 1:
						colorV.readFrame( &colorFrame );

						if ( colorFrame.isValid() ){
							if(firstColor == true)	firstColor = false;
							colorcv.data = (uchar*) colorFrame.getData();
							cv::cvtColor( colorcv, colorcv, CV_BGR2RGB );
							//colorBuffer = colorcv.clone();
							cv::flip(colorcv, colorBuffer, 1);
							cv::imshow( "RGB", colorBuffer );
							rgbFrameCount++;
						}
						break;

					default:
						puts( "Error retrieving a stream" );
				}
            
			}

			if (!firstDepth && !firstColor) {
				if ((rgbFrameCount > 125) && (rgbFrameCount > lastRgbFrameCount)) {
					if ((rgbFrameCount % frameGap) == 0) {
						char rgbChar[256], depthChar[256];

						if (argc > 2 && !strcmp(argv[1], "library")) {
							//debugging inputs: [library] [item] [surface(AB, AC, BC)] [Upright?]
							//if (!strcmp(argv[1], "library")) {
							//if(argv[3] == '\0' || argv[4] == '\0'){
							//}
								sprintf(rgbChar, "%s/rgb/%s_%s_R%04d.png", outputAddress.c_str(), argv[3], argv[4], outputFrameCount);
								sprintf(depthChar, "%s/depth/%s_%s_R%04d.png", outputAddress.c_str(), argv[3], argv[4], outputFrameCount);
							//}
						}else{
							sprintf(rgbChar, "%s/rgb/%04d.png", outputAddress.c_str(), outputFrameCount);
							sprintf(depthChar, "%s/depth/%04d.png", outputAddress.c_str(), outputFrameCount);
						}

						cv::imwrite(std::string(rgbChar), colorBuffer);
						cv::imwrite(std::string(depthChar), depthBuffer);

						printf("Frameset (%d) written to file..\n", outputFrameCount);

						lastRgbFrameCount = rgbFrameCount;
						outputFrameCount++;
					}

					if ((rgbFrameCount % frameGap) < 5) {
						captureMat = cv::Scalar(0.0, 255.0, 0.0);
					} else captureMat = cv::Scalar(0.0, 0.0, 255.0);
				}
				cv::imshow("ready", captureMat);

				int key = cv::waitKey(10) % 255;

				if (key == 27 || key == 'q') break;
			}
		}

	}

	cv::destroyWindow( "RGB" );
	cv::destroyWindow( "Depth" );
	depthV.stop();
	depthV.destroy();
	colorV.stop();
	colorV.destroy();
	device.close();
	OpenNI::shutdown();

    return 0;
}

int listdir(const char *path, std::vector<std::string>& depths, std::vector<std::string>& rgbs) {
   // std::vector<std::string> depthsTmp;
	struct dirent *entry;
    DIR *dp;

    dp = opendir(path);
    if (dp == NULL) {
        perror("opendir: Path does not exist or could not be read.");
        return -1;
    }

    while ((entry = readdir(dp))){
        //puts(entry->d_name);
		if(strcmp(entry->d_name, "depth") == 0){
			std::string newPath = path;
			newPath = newPath + "/depth";
			getImFiles(newPath.c_str(), depths);
			//depths = depthsTmp;
		}
		if(strcmp(entry->d_name, "rgb") == 0){
			std::string newPath = path;
			newPath = newPath + "/rgb";
			getImFiles(newPath.c_str(), rgbs);
			//depths = depthsTmp;
		}
	}

    closedir(dp);
    return 0;
}

int getImFiles(const char *path, std::vector<std::string>& names) {
	//std::vector<std::string> namesTmp;
    struct dirent *entry;
    DIR *dp;

    dp = opendir(path);
    if (dp == NULL) {
        perror("opendir: Path does not exist or could not be read.");
        return -1;
    }

    while ((entry = readdir(dp))){
        //puts(entry->d_name);
		if(strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0){
			std::string imgName = path;
			//char* imgName1 = entry->d_name;
			imgName = imgName + "/" + entry->d_name;
			names.push_back(imgName);
		}
	}

    closedir(dp);
	//names = namesTmp;
    return 0;
}
