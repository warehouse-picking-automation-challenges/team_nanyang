#include <iostream>

#include <string>
#include <vector>
#include <dirent.h>

#include "VirtualKinect.h"
#include "RGBD_Capture.h"
//#include <OpenNI.h>
#include <opencv2/opencv.hpp>

int iLowH = DEFAULT_I_LOW_H;
int iHighH = DEFAULT_I_HIGH_H;

int iLowS = DEFAULT_I_LOW_S; 
int iHighS = DEFAULT_I_HIGH_S;

int iLowV = DEFAULT_I_LOW_V;
int iHighV = DEFAULT_I_HIGH_V;

int iNearD = DEFAULT_I_NEAR_D;
int iFarD = DEFAULT_I_FAR_D;

int bDepth = DEFAULT_BIN_DEPTH_MM-300;

cv::Rect roiRect;

bool bDevelopmentMode_ = true;
bool drawing_box = false;

int listdir(const char *path, std::vector<std::string>& depths, std::vector<std::string>& rgbs);
int getImFiles(const char *path, std::vector<std::string>& names);
void onMouse(int event, int x, int y, int flag, void* param);
bool loadParameters(std::string filename);
bool saveParameters(std::string filename);
int getImFiles(const char *path, std::vector<std::string>& names);
bool isDepthValid(cv::Mat& depth);

class RGBDCapture_mod : public RGBDCapture {
	friend class RGBDCapture;
public:
	inline void setParams(int iLowH_, int iHighH_, int iLowS_, int iHighS_, int iLowV_, int iHighV_, int iNearD_, int iFarD_) { 
		iLowH = iLowH_;
		iHighH = iHighH_;
		iLowS = iLowS_;
		iHighS = iHighS_;
		iLowV = iLowV_;
		iHighV = iHighV_;
		iNearD = iNearD_;
		iFarD = iFarD_;
	}
	
	inline void setROI(cv::Rect rect) { roiRect_ = rect; }
	inline void setBinDepth(float bd) { binDepth_ = bd; bUsePreloadBinDepth_ = false; }

	inline bool generateMask_public(const cv::Mat& src, const cv::Mat& depth, cv::Mat& dst, int iBlobs) { 
		bool retVal = generateMask(src, depth, dst, iBlobs);
		return retVal;
	}
};

int main (int argc, char* argv[])
{
	std::string inputFolder;
	if (argc > 1) {
		inputFolder = std::string(argv[1]);
		std::cout << "Processing data from: " << inputFolder.c_str() << std::endl;
	} else {
		std::cout << "ERROR. No input directory provided as command line argument." << std::endl;
		return 1;
	}

	// Determine input type (library or test_data)
	// If directory contains rgb, assume test data, otherwise assume contains item name and then rgbd then rgb etc
	bool isLibraryData = true;
	{
		struct dirent *entry;
		DIR *dp;

		std::string rgbDir = inputFolder + "/rgb";

		dp = opendir(rgbDir.c_str());
		if (dp == NULL) {
			printf("Looks like it's library data.\n");
			inputFolder = inputFolder + "/rgbd";
		} else {
			printf("Looks like it's test data.\n");
			isLibraryData = false;
		}
	}

    loadParameters(_OUTPUT_DIRECTORY_ "/parameters/param.yml");

    if (bDevelopmentMode_) {
        cv::namedWindow("Control_HSV", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Control_Depth", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("image", CV_WINDOW_AUTOSIZE);

        //Create trackbars in "Control" windows
        cvCreateTrackbar("LowH", "Control_HSV", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control_HSV", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control_HSV", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control_HSV", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control_HSV", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control_HSV", &iHighV, 255);

        cvCreateTrackbar("NearD", "Control_Depth", &iNearD, 1500);
        cvCreateTrackbar("FarD", "Control_Depth", &iFarD, 1500);

		cvCreateTrackbar("BinDepth", "Control_Depth", &bDepth, 300);

        cv::setMouseCallback("image", onMouse);
    }

	roiRect.width = 0;
	roiRect.height = 0;

	int currentBlobCount = -1;

	std::vector<std::string> rgbIms, depthIms;
	if (listdir(inputFolder.c_str(), depthIms, rgbIms) != 0) std::cout << "Error. Something wrong." << std::endl;

	// Go through and count blobs if labels are available, and find RGB names and rename Depths to match
	std::vector<int> blobCounts;

	RGBDCapture_mod rgbd_cap;

	for (int i = 0; i < rgbIms.size(); i++) {	

		if (!isLibraryData) {
			struct dirent *entry;
			DIR *dp;

			std::string labelsFolder = inputFolder + "/labels";

			dp = opendir(labelsFolder.c_str());
			if (dp != NULL) {
				std::vector<std::string> labelsFileNames;
				while ((entry = readdir(dp))){
					//puts(entry->d_name);
					if(strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0){
						std::string labelsName = labelsFolder + "/" + entry->d_name;
						labelsFileNames.push_back(labelsName);
					}
				}

				std::sort(labelsFileNames.begin(), labelsFileNames.end());

				for (int iii = 0; iii < labelsFileNames.size(); iii++) {
					// Read in each file and count how many blobs are in the image
					std::ifstream fs(labelsFileNames.at(iii).c_str());

					int count = 0;
					if (fs.is_open()) {
						do 
						{
							std::string line;
							std::getline(fs, line);

							if (line.empty()) break;
							count++;
						} while (1);
					}
					blobCounts.push_back(count);
				}

			}
		}

	}

	std::string maskFolder = inputFolder + "/mask";
	mkdir(maskFolder.c_str());
    
	int i = 0;
	bool showingPreloadedMask = true;

	cv::Mat color, depth, depthShow, colorShow;

	roiRect = cv::Rect(640/4, 480 * 0.3, 640/2, 480/2);

	while (i < rgbIms.size()) {
		
		cv::Mat mask;

		unsigned found = rgbIms[i].find_last_of("/\\");

		if (found == std::string::npos) break;
		std::string maskFile = maskFolder + "/" + rgbIms[i].substr(found+1);
        
		if (showingPreloadedMask) {
			mask = cv::imread(maskFile, CV_LOAD_IMAGE_GRAYSCALE);
			if (mask.rows == 0) showingPreloadedMask = false;
		}

		if (color.rows == 0) { // Only read in raw data if it hasn't been read in yet
			std::string colorFile = rgbIms[i];
			color = cv::imread(colorFile, CV_LOAD_IMAGE_COLOR);
			if (color.rows == 0) {
				printf("%s << ERROR! Unable to read image #%d (%s)\n", __FUNCTION__, i, colorFile.c_str());
				break;
			}
			std::string depthFile = depthIms[i];
			depth = cv::imread(depthFile, CV_LOAD_IMAGE_ANYDEPTH); //depth = cv::imread(depthFile, CV_LOAD_IMAGE_GRAYSCALE);
			if (isDepthValid(depth)) {
				depth /= 5;
				depth.convertTo(depthShow, CV_8UC1, 0.05f);
			} else depth = cv::Mat();
		}

		if (currentBlobCount == -1) currentBlobCount = blobCounts.size() == 0 ? (isLibraryData ? 1 : 2) : blobCounts.at(i);
		
		if (!showingPreloadedMask) {
			rgbd_cap.setParams(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iNearD, iFarD);
			rgbd_cap.setROI(roiRect);
			rgbd_cap.setBinDepth(float(bDepth+300)/1000.0);
			rgbd_cap.setBinName("bin_J");
			rgbd_cap.generateMask_public(color, depth, mask, currentBlobCount);
			if (mask.rows == 0) mask = cv::Mat::zeros(color.size(), CV_8UC1);
			roiRect = rgbd_cap.getROI();
			if (currentBlobCount == 0) mask *= 0;
		}

		std::vector<cv::Mat> rgb;
		cv::split(color, rgb);
		rgb[0] += depthShow;
		cv::merge(rgb, color);

		if (depth.rows > 0) cv::imshow("depth", depthShow);
		color.copyTo(colorShow);
		cv::rectangle(colorShow, roiRect, CV_RGB(255,0,0));
		cv::imshow("image", colorShow);
		cv::imshow("mask", mask);

		int key = cv::waitKey(30) % 255;

		if (key == 'y') { // Y BUTTON TO APPROVE OF THE CURRENT MASK DISPLAYED TO SCREEN
			cv::imwrite(maskFile, mask);
			currentBlobCount = -1;
			showingPreloadedMask = true;
			color = cv::Mat();
			depth = cv::Mat();
			depthShow = cv::Mat();
			i++;
		} else if (key == 38) { // UP ARROW TO INCREASE NUMBER OF BLOBS
			currentBlobCount++;
			showingPreloadedMask = false;
		} else if (key == 40) { // DOWN ARROW TO DECREASE NUMBER OF BLOBS
			currentBlobCount = std::max(0, currentBlobCount-1);
			showingPreloadedMask = false;
		} else if (key == 'n') { // N BUTTON TO REJECT MASK IMAGE THAT WAS ALREADY FOUND IN DIRECTORY
			showingPreloadedMask = false;
		} else if (key == 'r') { // R BUTTON TO RESET FILTER PARAMETERS
			iLowH = DEFAULT_I_LOW_H;
			iHighH = DEFAULT_I_HIGH_H;
			iLowS = DEFAULT_I_LOW_S;
			iHighS = DEFAULT_I_HIGH_S;
			iLowV = DEFAULT_I_LOW_V;
			iHighV = DEFAULT_I_HIGH_V;
			iNearD = DEFAULT_I_NEAR_D;
			iFarD = DEFAULT_I_FAR_D;
			
			cvSetTrackbarPos("LowH", "Control_HSV", iLowH);
			cvSetTrackbarPos("HighH", "Control_HSV", iHighH);
			cvSetTrackbarPos("LowS", "Control_HSV", iLowS);
			cvSetTrackbarPos("HighS", "Control_HSV", iHighS);
			cvSetTrackbarPos("LowV", "Control_HSV", iLowV);
			cvSetTrackbarPos("HighV", "Control_HSV", iHighV);
			cvSetTrackbarPos("NearD", "Control_Depth", iNearD);
			cvSetTrackbarPos("FarD", "Control_Depth", iFarD);
			cvSetTrackbarPos("binD", "Control_Depth", bDepth);
		} else if (key == 27 || key == 'q') { // Q BUTTON TO QUIT
			break;
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

        result = true;
    }

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

        result = true;
    }

    return result;
}

int listdir(const char *path, std::vector<std::string>& depths, std::vector<std::string>& rgbs) {
   // std::vector<std::string> depthsTmp;
	struct dirent *entry;
    DIR *dp;

    dp = opendir(path);
    if (dp == NULL) {
        cout << "opendir: Path <" << std::string(path) << "> does not exist or could not be read." << endl;
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

bool isDepthValid(cv::Mat& depth) {

	if (depth.type() != CV_16UC1) return false;

	for (int iii = 0; iii < depth.rows; iii++) {
		for (int jjj = 0; jjj < depth.cols; jjj++) {
			if ((depth.at<unsigned short>(iii,jjj) % 5) != 0) return false;
		}
	}

	return true;
}
