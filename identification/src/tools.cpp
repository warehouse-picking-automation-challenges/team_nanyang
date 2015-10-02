#include "tools.h"

#undef max
#undef min

void writeMatrixToCSV(const cv::Mat& src, string filename, vector<string> objectNames) {
	int precision = 2;
	ofstream output_csv;
	output_csv.open(filename.c_str());

	if (src.dims == 2) {
		if (objectNames.size() > 0) { // Write out object indices in top row
			for (int iii = 0; iii < objectNames.size(); iii++) {
				output_csv << "," << iii+1;
			}
			output_csv << endl;
		}
		for (int iii = 0; iii < src.rows; iii++) {
			if (objectNames.size() > 0) output_csv << (iii+1) << ":" << objectNames.at(iii) << ",";
			for (int jjj = 0; jjj < src.cols-1; jjj++) {
				output_csv << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << src.at<double>(iii,jjj) << ",";
			}
			output_csv << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << src.at<double>(iii,src.cols-1) << endl;
		}
	} else if (src.dims == 3) {
		for (int iii = 0; iii < src.size[0]; iii++) {
			if (objectNames.size() > 0) { // Write out object indices in top row
				for (int jjj = 0; jjj < objectNames.size(); jjj++) {
					output_csv << "," << jjj+1;
				}
				output_csv << endl;
			}
			
			for (int jjj = 0; jjj < src.size[1]; jjj++) {
				if (objectNames.size() > 0) output_csv << (jjj+1) << ":" << objectNames.at(jjj) << ",";
				for (int kkk = 0; kkk < src.size[2]-1; kkk++) {
					output_csv << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << src.at<double>(iii,jjj,kkk) << ",";
				}
				output_csv << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << src.at<double>(iii,jjj,src.size[2]-1) << endl;
			}
			output_csv << endl;
		}
	}
	
	output_csv.close();
}

void generateIdentityConfigurations(vector<cv::Mat>& out, int blob_count, int class_count) {
	if (class_count == -1) class_count = blob_count;

	vector<int> seq;
	for (int iii = 1; iii <= blob_count; iii++) seq.push_back(iii);

	vector<vector<int> > super_vec;
	generatePermutations(blob_count, seq, super_vec);
	makePermutationsUnique(super_vec);

	for (int iii = 0; iii < super_vec.size(); iii++) {
        cv::Mat cfg = cv::Mat::zeros(blob_count, class_count, CV_64FC1);
		
        for (int jjj = 0; jjj < super_vec.at(iii).size(); jjj++) {
           cfg.at<double>(jjj, super_vec.at(iii).at(jjj)-1) = 1.0; 
		}
		out.push_back(cfg);
		//cout << "cfg (" << iii << ") = " << endl << cfg << endl;
	}

}

bool is_contained(int val, vector<int>& arr) {

	for (int iii = 0; iii < arr.size(); iii++) {
		if (arr.at(iii) == val) return true;
	}
	return false;
}

void generatePermutations(int n, vector<int>& A, vector<vector<int> >& super_vec) {
	if (n != 1) {
		for (int iii = 1; iii <= n; iii++) {
			generatePermutations(n-1, A, super_vec);
			int j;
			((n % 2) == 1) ? j = 1 : j = iii;
			int tmp = A.at(j-1);
			A.at(j-1) = A.at(n-1);
			A.at(n-1) = tmp;
		}
	} else super_vec.push_back(A);
}

void makePermutationsUnique(vector<vector<int> >& super_vec) {
	for (int iii = 0; iii < super_vec.size(); iii++) {
        int jjj = iii+1;

        while (jjj < super_vec.size()) {
            bool is_same = true;
            for (int kkk = 0; kkk < super_vec.at(iii).size(); kkk++) {
               if (super_vec.at(iii).at(kkk) != super_vec.at(jjj).at(kkk)) is_same = false;
			}
            
            if (is_same == true) {
                super_vec.erase(super_vec.begin()+jjj);
			} else jjj = jjj + 1;
		}
	}

}

double calculateProbabilityAccuracy(int elements, double *limits, int tallies[][2]) {
	
	int min_count = 0;
	double modelAccuracy = 0.0, totalBinWeight = 0.0;

	for (int iii = 0; iii < elements; iii++) {
		min_count += tallies[iii][0] + tallies[iii][1];
	}
	min_count /= (elements*10);
	
	for (int iii = 0; iii < elements; iii++) {
		double expectedRate = (iii == 0) ? (1.0 + limits[iii])/2.0 : (limits[iii-1]+limits[iii])/2.0;
		double measuredRate = double(tallies[iii][0]) / double(tallies[iii][1] + tallies[iii][0]);
		
		int binCount = (tallies[iii][1] + tallies[iii][0]);

		if (binCount > min_count) {
			modelAccuracy += double(binCount) * abs(expectedRate-measuredRate);
			totalBinWeight += double(binCount);
		}
	}

	modelAccuracy /= totalBinWeight;
	modelAccuracy = 1-modelAccuracy;

	return modelAccuracy;

}

bool readRotationFromTxtFile(ifstream& rotationsFile, vector<double>& rotation) {
	for (int kkk = 0; kkk < 9; kkk++) {
		double val;
		rotationsFile >> val;
		rotation.push_back(val);
	}
	return true;
}

bool readRotationFromAlnFile(ifstream& rotationsFile, vector<double>& rotation) {

	string currentLine;
	int hashFound = 0;
	
	while (hashFound < 2) {
		std::getline(rotationsFile, currentLine);
		if (currentLine.size() > 0) {
			if (currentLine.at(0) == '#') hashFound++;
		}
	}

	for (int kkk = 0; kkk < 12; kkk++) {
		double val;
		rotationsFile >> val;
		if ((kkk % 4) != 3) rotation.push_back(val);
	}

	return true;
}

void convertProbabilityMat(const cv::Mat& src, cv::Mat& dst, bool setAbsentSelfTestsToZero) {

	if (&src == NULL) return;
	if (src.empty()) return;
	
	src.copyTo(dst);
	
	if (src.dims == 2) {
		for (int iii = 0; iii < src.rows; iii++) {
			for (int jjj = 0; jjj < src.cols; jjj++) {
				if (src.at<double>(iii,jjj) == -1.0) dst.at<double>(iii,jjj) = 0.0;
				if (src.at<double>(iii,jjj) == -2.0) dst.at<double>(iii,jjj) = setAbsentSelfTestsToZero? 0.0 : 1.0;
			}
		}
	} else if (src.dims == 3) {
		for (int iii = 0; iii < src.size[0]; iii++) {
			for (int jjj = 0; jjj < src.size[1]; jjj++) {
				for (int kkk = 0; kkk < src.size[2]; kkk++) {
					if (src.at<double>(iii,jjj,kkk) == -1.0) dst.at<double>(iii,jjj,kkk) = 0.0;
					if (src.at<double>(iii,jjj,kkk) == -2.0) dst.at<double>(iii,jjj,kkk) = setAbsentSelfTestsToZero ? 0.0 : 1.0;
				}
				
			}
		}
	}
}

cv::Mat generateMask(const cv::Mat& src, int blobs) {
	// http://opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html

	bool developmentMode = false;

	cv::Mat preDest, dst, imgHSV, workingMask;
	preDest = cv::Mat::zeros(src.size(), CV_8UC3);

	cv::Rect subset(int(src.cols/6), int(src.rows/6), int(src.cols/1.5), int(src.rows/1.5));
	cv::Mat rectMask(src.size(), CV_8UC1, cv::Scalar::all(0));
	rectMask(subset).setTo(cv::Scalar::all(255));

	cvtColor(src, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	if (developmentMode) cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0; 
	int iHighS = 255;

	int iLowV = 170;
	int iHighV = 255;

	int iLowC = 500;
	int iHighC = 10000;

	int kernel = 5;

	int maxRatio = 5;


	if (developmentMode) { //Create trackbars in "Control" window

		cvCreateTrackbar("kernel", "Control", &kernel, 20);
		cvCreateTrackbar("ratio", "Control", &kernel, 5);

		cvCreateTrackbar("LowC", "Control", &iLowC, 10000);
		cvCreateTrackbar("HighC", "Control", &iHighC, 10000);

		cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control", &iHighV, 255);
		
		cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control", &iHighH, 179);

		cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	}

	vector<vector<cv::Point> > contours;

	while (1) {
		inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), workingMask); //Threshold the image
      
		//morphological opening (remove small objects from the foreground)
		erode(workingMask, workingMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel)) );
		dilate( workingMask, workingMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel)) ); 

		//morphological closing (fill small holes in the foreground)
		dilate( workingMask, workingMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel)) ); 
		erode(workingMask, workingMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel)) );

		
		vector<cv::Vec4i> hierarchy;
		double contour_area;
		cv::Mat temp_image;

		for (int iii = 0; iii < workingMask.rows; iii++) {
			for (int jjj = 0; jjj < workingMask.cols; jjj++) {
				if ((iii == 0) || (iii == workingMask.rows-1) || (jjj == 0) || (jjj == workingMask.cols-1)) {
					workingMask.at<unsigned char>(iii,jjj) = 0;
				}
			}
		}

		// find all contours in the binary image
		contours.clear();
		cv::Mat bak;
		workingMask.copyTo(bak);
		findContours(workingMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

		// Find indices of contours whose area is less than `threshold` 
		if (!contours.empty()) {
			for (size_t i=0; i<contours.size(); ++i) {
				contour_area = contourArea(contours[i]) ;
				if ((contour_area > double(iHighC)) || (contourRatio(contours.at(i)) > maxRatio)) {
					cv::drawContours(workingMask, contours, int(i), cv::Scalar(0.0, 0.0, 0.0), CV_FILLED );
					contours.erase(contours.begin()+i);
					i--;
				}
			}
		}

		while (contours.size() > blobs) {
			double min_area = 9e99;
			int min_index = -1;
			for (size_t i=0; i< contours.size(); ++i) {
				double contour_area = contourArea(contours[i]) ;
				if (contour_area < min_area) {
					min_index = int(i);
					min_area = contour_area;
				}
			}
			contours.erase(contours.begin()+min_index);
		}

		//workingMask.copyTo(temp_image, rectMask);

		if (!contours.empty()) {
			for (size_t i=0; i<contours.size(); ++i) {
				cv::drawContours(preDest, contours, int(i), cv::Scalar(255.0*double(i+1)/double(contours.size()), 255.0*double(i+1)/double(contours.size()), 255.0*double(i+1)/double(contours.size())), CV_FILLED );
			}
		}

		if (developmentMode) {
			cv::imshow("Auto-masking Original", src);
			cv::imshow("Auto-masking Mask", preDest);
			cv::waitKey(1);
		} else break;

	}

	(preDest.type() != CV_8UC1) ? cvtColor(preDest, dst, cv::COLOR_RGB2GRAY) : preDest.copyTo(dst);
	//preDest.convertTo(dst, CV_8UC1);
	return dst;

}

double contourRatio(const vector<cv::Point>& contour) {
	
	if (contour.size() == 0) return -1.0;

	int min_x = contour.at(0).x;
	int max_x = contour.at(0).x;
	int min_y = contour.at(0).y;
	int max_y = contour.at(0).y;

	for (int iii = 1; iii < contour.size(); iii++) {
		min_x = min(min_x, contour.at(iii).x);
		max_x = max(max_x, contour.at(iii).x);
		min_y = min(min_y, contour.at(iii).y);
		max_y = max(max_y, contour.at(iii).y);
	}

	int width = max_x - min_x;
	int height = max_y - min_y;

	return max(double(width)/double(height), double(height)/double(width));
}

bool duplicateString(const vector<string>& string_vec) {
	
	if (string_vec.size() <= 1) return false;

	for (int iii = 0; iii < string_vec.size()-1; iii++) {
		for (int jjj = iii+1; jjj < string_vec.size(); jjj++) {
			if (string_vec.at(iii) == string_vec.at(jjj)) return true;
		}
	}

	return false;
}

int float2int(float val) { 
	if (val >= 0.0) {
		return int(floor(val + 0.5)); 
	} else {
		return int(ceil(val - 0.5));
	}
}

void makeDirectory(string dir_name) {
#ifdef _IS_WINDOWS_
    CreateDirectory (dir_name.c_str(), NULL);
#else
    mkdir(dir_name.c_str(), 0777);
#endif
}

bool getContents(string src, vector<string>& contents) {

    char inputDirectory[256];
    sprintf_s(inputDirectory, "%s", src.c_str());

#ifdef _IS_WINDOWS_
    if (inputDirectory[_tcslen(inputDirectory) - 1] != '\\') _tcscat_s(inputDirectory, _T("\\"));
    _tcscat_s(inputDirectory, _T("*.*"));

    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(inputDirectory, &ffd);

    if (hFind == INVALID_HANDLE_VALUE) {
        std::printf("%s << Invalid handle value. Perhaps the address specified (%s) is wrong?\n", __FUNCTION__, src.c_str());
        return false;
    }

    do {
		if (std::string(ffd.cFileName).size() < 3) continue;
		if (std::string(ffd.cFileName) == "Note.txt") continue;
		if (std::string(ffd.cFileName) == "Thumbs.db") continue;
        contents.push_back(std::string(ffd.cFileName));
    } while (FindNextFile(hFind, &ffd));
#else
    DIR* dirFile = opendir( src.c_str() );
    if ( dirFile )
    {
        struct dirent* hFile;
        bool gIgnoreHidden = true;
        while (( hFile = readdir( dirFile )) != NULL )
            {
            if ( !strcmp( hFile->d_name, "."  )) continue;
            if ( !strcmp( hFile->d_name, ".." )) continue;
			if ( !strcmp( hFile->d_name, "Note.txt" )) continue;
			if ( !strcmp( hFile->d_name, "Thumbs.db" )) continue;

            // in linux hidden files all start with '.'
            if ( gIgnoreHidden && ( hFile->d_name[0] == '.' )) continue;

            // dirFile.name is the name of the file.
            contents.push_back(std::string(hFile->d_name));
        }
        closedir( dirFile );
    }
#endif

	return true;
}

bool hasExtension(string& src, string& ext) {
	
	if (src.size() <= ext.size()+1) return false;
	if (src.at(src.size()-(ext.size()+1)) != '.') return false;
	
	for (int iii = 0; iii < ext.size(); iii++) if (src.at(src.size()-(ext.size())+iii) != ext.at(iii)) return false;

	return true;
}

void filterContents(vector<string>& contents, const vector<int>& elevation_indices, bool is_mask) {

    for (int iii = 0; iii < contents.size(); iii++) {

        if (is_mask) {
            size_t index = 0;
            index = contents.at(iii).find("_mask", index);
            if (index != string::npos) contents.at(iii).replace(index, 5, ".mask");
        }

        if (contents.at(iii)[0] == '.') {
            contents.erase(contents.begin()+iii);
            iii--;
			continue;
        } 
		
		if (contents.at(iii).size() <= 3) {
            contents.erase(contents.begin()+iii);
            iii--;
			continue;
        } 
		
		if (contents.at(iii).size() > 4) {
		    string txt_string = "txt";
            if (hasExtension(contents.at(iii), txt_string)) {
                contents.erase(contents.begin()+iii);
                iii--;
				continue;
            }
		}

        if (elevation_indices.size() > 0) {
			if (elevation_indices.at(0) != -1) {
				int index = getNPVal(contents.at(iii)); // need to fill with NP number

				bool permitted = false;
				for (int jjj = 0; jjj < elevation_indices.size(); jjj++) {
					if (index == elevation_indices.at(jjj)) {
						permitted = true;
						break;
					}
				}
				if (!permitted) {
					contents.erase(contents.begin()+iii);
					iii--;
				}
			}
        }
        //if ( (ffd.cFileName[0] != 'A') || (ffd.cFileName[1] != 'P') || (ffd.cFileName[2] != 'C') || (ffd.cFileName[3] != '_') ) continue;
    }
}

void displayKeyPoints(const cv::Mat& image, const vector<cv::KeyPoint>& fPoints, const vector<cv::KeyPoint>& ePoints, cv::Mat& outImg, int thickness, bool pointsOnly) {

    cv::Scalar colours[2];
	colours[0] = cv::Scalar(0,0,255);
	colours[1] = cv::Scalar(0,255,0);

	vector<cv::KeyPoint> pts_vecs[2];
	pts_vecs[0] = fPoints;
	pts_vecs[1] = ePoints;
    
	image.copyTo(outImg);

    cv::Point centerPt;

    int radius, crossLength;

    if (thickness == 0) thickness = 1;

	for (int n = 0; n < 2; n++) {
		for (unsigned int i = 0; i < pts_vecs[n].size(); i++) {
			centerPt = pts_vecs[n].at(i).pt;

			centerPt.x = int(pts_vecs[n].at(i).pt.x * 16.0);
			centerPt.y = int(pts_vecs[n].at(i).pt.y * 16.0);

			radius = int(16.0 * (pts_vecs[n].at(i).size/2.0));
			crossLength = int(2 * 16.0);
        
			if (pointsOnly) {
#ifdef CV_AA
				circle(outImg, centerPt, 1, colours[n], 2, CV_AA, 4);
#else
				circle(outImg, centerPt, 1, colours[n], 2, cv::LINE_AA, 4);
#endif
			} else {
				 if (radius > 0) {
#ifdef CV_AA
					circle(outImg, centerPt, radius, colours[n], thickness, CV_AA, 4);
#else
					circle(outImg, centerPt, radius, colours[n], thickness, cv::LINE_AA, 4);
#endif
				}

#ifdef CV_AA
				line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), colours[n], thickness, CV_AA, 4);
				line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), colours[n], thickness, CV_AA, 4);
#else
				line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), colours[n], thickness, cv::LINE_AA, 4);
				line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), colours[n], thickness, cv::LINE_AA, 4);
#endif

			}
		}
	}
	
    return;
}

cv::Mat generateCombinedMask(const cv::Mat& src) {
	cv::Mat dst = cv::Mat::zeros(src.size(), CV_8UC1);
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			if (src.at<unsigned char>(iii,jjj) > 0) dst.at<unsigned char>(iii,jjj) = 255;
		}
	}
	return dst;
}

cv::Mat generateSpecificMask(const cv::Mat& src, int index) {
	
	cv::Mat dst = cv::Mat::zeros(src.size(), CV_8UC1);

	vector<unsigned char> uniqueLevels;
	
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			bool isUnique = true;
			for (int kkk = 0; kkk < uniqueLevels.size(); kkk++) {
				if (src.at<unsigned char>(iii,jjj) == uniqueLevels.at(kkk)) isUnique = false;
			}
			if (isUnique) uniqueLevels.push_back(src.at<unsigned char>(iii,jjj));
		}
	}

	if (uniqueLevels.size() == 0) {
		std::printf("%s << ERROR! Image size = (%d, %d)\n", __FUNCTION__, src.rows, src.cols);
		return cv::Mat();
	}

	sort(uniqueLevels.begin(), uniqueLevels.end());

	int desiredValue = uniqueLevels.at(1 + index);

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			if (src.at<unsigned char>(iii,jjj) == desiredValue) dst.at<unsigned char>(iii,jjj) = 255;
		}
	}

	return dst;
}

void createMatchingMatrix(cv::Mat& matchingMatrix, const cv::Mat& desc1, const cv::Mat& desc2) {

	if ((desc1.rows == 0) || (desc2.rows == 0)) return;

    vector<vector<cv::DMatch> > matches1to2, matches2to1;
	cv::Ptr<cv::DescriptorMatcher> dMatcher;

	if (desc1.type() == CV_8UC1) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	} else if ((desc1.type() == CV_64FC1) || (desc1.type() == CV_32FC1)) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce");
	} else {
		std::printf("%s << ERROR! desc1.type() (%d) unrecognized.\n", __FUNCTION__, desc1.type());
	}

	dMatcher->knnMatch( desc1, desc2, matches1to2, desc1.rows );
	dMatcher->knnMatch( desc2, desc1, matches2to1, desc2.rows );

    matchingMatrix = cv::Mat::zeros(int(matches1to2.size()), int(matches2to1.size()), CV_64FC1);

    cv::Mat countMat = cv::Mat::zeros(int(matches1to2.size()), int(matches2to1.size()), CV_64FC1);

    // IM 1 to IM 2
    for (unsigned int iii = 0; iii < matches1to2.size(); iii++) {
        for (unsigned int jjj = 0; jjj < matches1to2[iii].size(); jjj++) {
            matchingMatrix.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += matches1to2.at(iii).at(jjj).distance;
            countMat.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += 1.0;
        }
    }

    // IM 2 to IM 1
    for (unsigned int iii = 0; iii < matches2to1.size(); iii++) {
        for (unsigned int jjj = 0; jjj < matches2to1[iii].size(); jjj++) {
            matchingMatrix.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += matches2to1.at(iii).at(jjj).distance;
            countMat.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += 1.0;
        }
    }

    matchingMatrix /= 2.0;

}

void twoWayPriorityMatching(cv::Mat& matchingMatrix, vector<vector<cv::DMatch> >& bestMatches) {

	bestMatches.clear();

    // void getPriorityScores(Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<vector<int> >& priorityMatches);

    // Potentially more efficient way of doing things:
        // Just search matrix for lowest match score, and record the next best in that 'cross' and blank out that cross,
        // then search for the lowest match score remaining in the matrix and so on and so forth...
        //
        // Actually many-many, 1-many, 1-1 can all use this basic approach, but just with different "blanking" strategies
        // e.g. 1-many would blank out the entire column, but not row
        // only many-many would not be able to record a valid "second best" score
            // and many-many would also have N^2 matches rather than just N for the others...

	bool rowsStillRemain = false;
	int remainingRows = 0;

	for (int iii = 0; iii < matchingMatrix.rows; iii++) {
		bool anyInThisRow = false;
		for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
			if (matchingMatrix.at<double>(iii,jjj) >= 0) {
				rowsStillRemain = true;
				anyInThisRow = true;
			}
		}

		if (anyInThisRow) remainingRows++;
	}

	while (rowsStillRemain) {

		double bestScore = std::numeric_limits<double>::max();
		int bestRow = -1, bestCol = -1;

		for (int iii = 0; iii < matchingMatrix.rows; iii++) {
			for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
				if ((matchingMatrix.at<double>(iii,jjj) <= bestScore) && (matchingMatrix.at<double>(iii,jjj) >= 0)) {
					bestScore = matchingMatrix.at<double>(iii,jjj);
					bestRow = iii;
					bestCol = jjj;
				}
			}
		}

		if ((bestScore < 0) || (bestScore == std::numeric_limits<double>::max())) {
			rowsStillRemain = false;
			break;
		}

		matchingMatrix.at<double>(bestRow,bestCol) = -1.0;

		cv::DMatch currentMatch;
		vector<cv::DMatch> currentMatchVector;

		currentMatch.queryIdx = bestRow;
		currentMatch.trainIdx = bestCol;

		double secondScore = std::numeric_limits<double>::max();

		for (int iii = 0; iii < matchingMatrix.rows; iii++) {
			if ((matchingMatrix.at<double>(iii,bestCol) <= secondScore) && (matchingMatrix.at<double>(iii,bestCol) >= 0)) {
				secondScore = matchingMatrix.at<double>(iii,bestCol);
			}
			matchingMatrix.at<double>(iii,bestCol) = -1;
		}

		for (int iii = 0; iii < matchingMatrix.cols; iii++) {
			if ((matchingMatrix.at<double>(bestRow,iii) <= secondScore) && (matchingMatrix.at<double>(bestRow,iii) >= 0)) {
				secondScore = matchingMatrix.at<double>(bestRow,iii);
			}
			matchingMatrix.at<double>(bestRow,iii) = -1;
		}

		// If it is literally the last match available, it won't have a second best score...
		if (secondScore == std::numeric_limits<double>::max()) secondScore = bestScore;

		currentMatch.distance = float(bestScore);
		
		currentMatchVector.push_back(currentMatch);
		bestMatches.push_back(currentMatchVector);
	}
}

void sortMatches(vector<vector<cv::DMatch> >& matches1to2) {
	vector<vector<cv::DMatch> > matchesCpy, newMatches;

	if (matches1to2.size() <= 1) return;

	matchesCpy.assign(matches1to2.begin(), matches1to2.end());

	while (matchesCpy.size() > 0) {
		double bestDistance = matchesCpy.at(0).at(0).distance;
		int bestIndex = 0;

		for (unsigned int iii = 0; iii < matchesCpy.size(); iii++) {

			if (matchesCpy.at(iii).at(0).distance <= bestDistance) {
				bestDistance = matchesCpy.at(iii).at(0).distance;
				bestIndex = iii;
			}
		}

		newMatches.push_back(matchesCpy.at(bestIndex));
		matchesCpy.erase(matchesCpy.begin() + bestIndex);
	}

	newMatches.swap(matches1to2);
}

void filterMatchesByClass(vector<vector<cv::DMatch> >& matches1to2, const std::vector<int> im1foreground, const std::vector<int> im1edge, const std::vector<int> im2foreground, const std::vector<int> im2edge) {

	if (matches1to2.size() == 0) return;

	// ...

}

void generateConfusionMatrix(const cv::Mat& success, const cv::Mat& failure, cv::Mat& dst) {
	dst = cv::Mat::zeros(success.size(), CV_64FC1);

	for (int iii = 0; iii < success.rows; iii++) {
		for (int jjj = 0; jjj < success.cols; jjj++) {
			dst.at<double>(iii,jjj) = success.at<double>(iii,jjj) / (success.at<double>(iii,jjj) + failure.at<double>(iii,jjj));
		}
	}

}

void runSimulatedIdentityTests(const vector<vector<vector<double> > >& scores, const vector<double>& confidenceFormula, cv::Mat& success, cv::Mat& failure, vector<vector<double> >& resultsRatios, int& tripleTestCount, int& tripleTestSuccess) {
	success = cv::Mat::zeros(cv::Size(int(scores.size()),int(scores.size())), CV_64FC1);
	failure = cv::Mat::zeros(success.size(), CV_64FC1);
	
	tripleTestCount = 0;
	tripleTestSuccess = 0;

	cv::Mat debugConfusionMatrix = cv::Mat::zeros(success.size(), CV_64FC1);

	for (int iii = 0; iii < scores.size(); iii++) { // For each object tested
		for (int jjj = 0; jjj < scores.at(iii).size(); jjj++) { // For each particular test image for that object

			for (int mmm = 0; mmm < scores.size(); mmm++) { // For each OTHER object tested
				if (iii == mmm) continue;
				for (int nnn = 0; nnn < scores.at(mmm).size(); nnn++) { // For each particular test image for that object

					cv::Mat bestScores = cv::Mat::zeros(3, 3, CV_64FC1);

					bestScores.at<double>(0,0) = scores.at(iii).at(jjj).at(iii);
					bestScores.at<double>(0,1) = scores.at(iii).at(jjj).at(mmm);

					bestScores.at<double>(1,0) = scores.at(mmm).at(nnn).at(iii);
					bestScores.at<double>(1,1) = scores.at(mmm).at(nnn).at(mmm);

					double targetRatio = bestScores.at<double>(0,0) / bestScores.at<double>(0,1) ;
					double contestRatio = bestScores.at<double>(1,0) / bestScores.at<double>(1,1);

					// If test object matches true identity better than the alternative one does
					(targetRatio < contestRatio) ? success.at<double>(iii,mmm) += 1.0 : failure.at<double>(iii,mmm) += 1.0;

					/*
					generateConfusionMatrix(success, failure, debugConfusionMatrix);
					expandAndDisplay(debugConfusionMatrix, "debugConfusionMatrix");
					cv::waitKey(10);
					*/

					for (int aaa = 0; aaa < scores.size(); aaa++) { // For each 3rd object tested
						if ((iii == mmm) || (iii == aaa) || (mmm == aaa)) continue;
						for (int bbb = 0; bbb < scores.at(aaa).size(); bbb++) { // For each particular test image for that object

							// Complete the 3 x 3 distance matrix
							bestScores.at<double>(0,2) = scores.at(iii).at(jjj).at(aaa);
							bestScores.at<double>(1,2) = scores.at(mmm).at(nnn).at(aaa);

							bestScores.at<double>(2,0) = scores.at(aaa).at(bbb).at(iii);
							bestScores.at<double>(2,1) = scores.at(aaa).at(bbb).at(mmm);
							bestScores.at<double>(2,2) = scores.at(aaa).at(bbb).at(aaa);

							// Find all the ratios suggesting appropriateness of correct label for each candidate
							double targetRatio3 = max(targetRatio, bestScores.at<double>(0,0) / bestScores.at<double>(0,2));
							double contestRatio3a = max(contestRatio, bestScores.at<double>(1,0) / bestScores.at<double>(1,2));
							double contestRatioX = bestScores.at<double>(2,0) / bestScores.at<double>(2,1);
							double contestRatio3b = max(contestRatioX, bestScores.at<double>(2,0) / bestScores.at<double>(2,2));
							
							double closestContestRatio = min(contestRatio3a, contestRatio3b);

							tripleTestCount++;

							if (targetRatio3 < closestContestRatio) tripleTestSuccess++;

							vector<double> resultsTriple(3);
							resultsTriple.at(0) = min(targetRatio3 / closestContestRatio, closestContestRatio / targetRatio3);
							resultsTriple.at(1) = calculateConfidence(resultsTriple.at(0), confidenceFormula);
							resultsTriple.at(2) = (targetRatio3 < closestContestRatio) ? 1.0 : 0.0;
							resultsRatios.push_back(resultsTriple);

						}
					}
				}
			}
		}
	}
}

int getNPVal(string input, int& num_1, int& num_2) {

	num_1 = -1;
	num_2 = -1;

	int index_1 = 0;
	int index_2 = 0;
	int index_3 = 0;

	index_1 = int(input.find("NP", index_1));
	index_2 = int(input.find("_", index_2));
	index_3 = int(input.find(".", index_3));

	num_1 = atoi(input.substr(index_1+2, index_2-index_1-1).c_str());
	num_2 = atoi(input.substr(index_2+1, index_3-index_2-1).c_str());

	return num_1;
}

void fillHoles(const cv::Mat& src, cv::Mat& dst, int fillType) {

	//Items with big holes in model/mask, like the plastic cups, are better filled by floodfill method. 
	//Items with big holes in mask: - take and toss straw cups (plastic cups) -squeaking eggs (Has big hole but because of empty space between balls and label. Not important.) -safety glasses

	//For items with small bits hanging out, erodeMasks should be set to false. Else, the initial opening can result in an unexpected hole being created by the morphology.
	//Items that can be affected: -take and toss straw cups -outlet plugs -safety glasses -sharpie pens (did not get to test fully as program gives handling exception after few frames.)

	//Thin items or items with thin parts that contain more features may be better filled using just closing morphology. May leave holes in mask but thin parts don't get erased.
	//Items that can be affected: -outlet plugs -munchkin bath duck -sharpie pens (did not get to test fully as program gives handling exception after few frames.)

	//Items that need model replaced: -pencil cup

	cv::Mat toFix;
	src.copyTo(toFix);
	cv::Mat fixed;
	
	if(fillType==0){	//Closing morphological operation - Dilate then erode. Close small holes in foreground.
		cv::Mat closed = cv::Mat(toFix.rows, toFix.cols, CV_8UC1);
		cv::dilate(toFix, closed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size (15, 15),cv::Point(7, 7)));//cv::Size (7, 7),cv::Point(3, 3)));
		cv::erode(closed, closed, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size (15, 15),cv::Point(7, 7)));//cv::Size (7, 7),cv::Point(3, 3)));
		fixed = closed;
	}
	
	else if(fillType==1){
		//Flood fill then close
		cv::Mat mask = cv::Mat::zeros(toFix.rows + 2, toFix.cols + 2, CV_8U); //floodfill requires dst image to be 2 pixels wider and taller than src image
		cv::floodFill(toFix, mask, cv::Point(0 ,0), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY); //apply floodfill on background, turning foreground(object) into zeros(black)
		fixed = (mask==0); //invert image to get filled white object
		cv::resize(fixed, fixed, toFix.size());
		cv::erode(fixed, fixed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));		//base 7 seems to return best result (visually) through trial and error
		cv::dilate(fixed, fixed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));		//opening then closing the image gets good small mask of object
		//imshow("preFixed", cropImage(fixed));
		cv::dilate(fixed, fixed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));		//closing then opening also gets good mask but can be a bit wider
		cv::erode(fixed, fixed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));
	}

	else if(fillType==2){
		
		//cv::Mat canny_output;
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;
		cv::Mat temp;

		/// Detect edges using canny
		//Canny( toFix, canny_output, 0, 1, 3 );
		//imshow("Canny", canny_output);
		//cv::waitKey(0);
		/// Find contours
		findContours( toFix, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		/// Draw contours
		cv::Mat drawing = cv::Mat::zeros( toFix.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
			{
			cv::Scalar color = cv::Scalar( rand()&255, rand()&255, rand()&255 );
			drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
			}

		//temp = (drawing!=0);	//Obtain black image with white contours. Without, each contour will have random colour.
		temp = drawing;
		//imshow("Contours", temp);
		//cv::waitKey(0);
		cv::Mat mask = cv::Mat::zeros(temp.rows + 2, temp.cols + 2, CV_8U); 
		cv::floodFill(temp, mask, cv::Point(0 ,0), 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY); 
		cv::resize(mask, mask, toFix.size());
		temp = (mask==0);
		cv::erode(temp, temp, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));
		cv::dilate(temp, temp, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));
		cv::dilate(temp, temp, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));
		cv::erode(temp, temp, cv::getStructuringElement(cv::MORPH_RECT, cv::Size (15, 15),cv::Point(7, 7)));
		fixed = temp;
	}

	else
	{
		std::printf("Invalid fillType.");
		fixed = cv::Mat::ones(toFix.rows, toFix.cols, CV_8UC1);
		fixed = fixed*255;
	}
	

	//imshow("fillHoles", src);
	//cv::waitKey(0);
	//imshow("fillHoles", fixed);
	//cv::waitKey(0);

	dst = fixed;
	
}

double featuresMatch(const cv::Mat& foreground_descs_1, const cv::Mat& edge_descs_1, const cv::Mat& foreground_descs_2, const cv::Mat& edge_descs_2) {

	bool use_edge = false;

	if ( ((foreground_descs_1.rows == 0) && (edge_descs_1.rows == 0)) || ((foreground_descs_2.rows == 0) && (edge_descs_2.rows == 0)) ) {
		return std::numeric_limits<double>::max();
	}

	if ((foreground_descs_1.rows == 0) || (foreground_descs_2.rows == 0)) use_edge = true;

	if (use_edge && ((edge_descs_1.rows == 0) || (edge_descs_2.rows == 0))) return std::numeric_limits<double>::max();

	// Get a 2-way matching score...
	std::vector< cv::DMatch > matches;
	std::vector< std::vector< cv::DMatch > > matchesVector;

	cv::Ptr<cv::DescriptorMatcher> dMatcher;

	if (foreground_descs_1.type() == CV_8UC1) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	} else if ((foreground_descs_1.type() == CV_64FC1) || (foreground_descs_1.type() == CV_32FC1)) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce");
	}

	if (!use_edge) {
		dMatcher->knnMatch( foreground_descs_1, foreground_descs_2, matchesVector, 1 );
	} else dMatcher->knnMatch( edge_descs_1, edge_descs_2, matchesVector, 1 );

	return (use_edge ? 2.0 : 1.0) * calculateScoreFromDMatchVector(matchesVector);	
	//return calculateScoreFromDMatchVector(matchesVector);	
}

double dimensionsMatch(double width_1, double height_1, double width_2, double height_2) {

	if ((width_1 == 0) || (height_1 == 0) || (width_2 == 0) || (height_2 == 0)) {
		return 1.0;
	}

	double width_factor = min(width_1/width_2, width_2/width_1);
	double height_factor = min(height_1/height_2, height_2/height_1);
	
	double retVal = 1.0 - (width_factor*height_factor);

	if ((retVal < 0.0) || (retVal > 1.0)) {
		cout << "[" << __FUNCTION__ << "] Error!" << endl;
	}

	return max(min(retVal, 1.0), 0.0);
}

double colourHistMatch(const cv::Mat& test_hist, const cv::Mat& lib_hist) {

	if (lib_hist.rows == 0) return std::numeric_limits<double>::max();

	int nonzeroCount_1 = cv::countNonZero(test_hist);
	int nonzeroCount_2 = cv::countNonZero(lib_hist);

	if ((nonzeroCount_1 == 0) || (nonzeroCount_2 == 0)) std::printf("%s << (%d, %d)\n", __FUNCTION__, nonzeroCount_1, nonzeroCount_2);

	int method = 3; // DEFINITELY LOOKS THE BEST!!!

	cv::Mat test_hist_cpy, lib_hist_cpy;
	test_hist.copyTo(test_hist_cpy);
	lib_hist.copyTo(lib_hist_cpy);

	bool ignoreZeros = false;

	if (ignoreZeros) {

		for( int i = 0; i < test_hist.size[0]; i++ ) {
			if (test_hist_cpy.dims > 1) {
				for( int j = 0; j < test_hist.size[1]; j++ ) {
					if (test_hist_cpy.dims > 2) {
						for (int k = 0; k < test_hist.size[2]; k++ ) {
							if ((i == 0) || (j == 0) || (k == 0)) {
								test_hist_cpy.at<float>(i,j,k) = 0.0;
								lib_hist_cpy.at<float>(i,j,k) = 0.0;
							}
						}
					} else {
						if ((i == 0) || (j == 0)) {
							test_hist_cpy.at<float>(i,j) = 0.0;
							lib_hist_cpy.at<float>(i,j) = 0.0;
						}
					}
				}
			} else {
				if (i == 0) {
					test_hist_cpy.at<float>(i) = 0.0;
					lib_hist_cpy.at<float>(i) = 0.0;
				}
			}
			
		}
	}

	return abs(cv::compareHist(test_hist_cpy, lib_hist_cpy, method));
}

void findBestScores(const vector<vector<double> >& matchingScores, vector<double>& scores, vector<int>& id1, vector<int>& id2, int num, int views_per_level) {

	scores.clear();
	id1.clear();
	id2.clear();

	for (int iii = 0; iii < num; iii++) {
		id1.push_back(-1);
		id2.push_back(-1);
		scores.push_back(std::numeric_limits<double>::max());
	}
	
	// For each rank
	for (int kkk = 0; kkk < num; kkk++) {

		// For each score
		for (int iii = 0; iii < matchingScores.size(); iii++) {
			for (int jjj = 0; jjj < matchingScores.at(iii).size(); jjj++) {

				// If the best for that rank
				if (matchingScores.at(iii).at(jjj) < scores[kkk]) {

					// Check if sufficiently distant for higher ranked
					bool sufficientDistance = true;
					vector<int> neighbours;
					findNeighbours(jjj, neighbours, views_per_level);

					for (int zzz = 0; zzz < kkk; zzz++) {
						vector<int> neighbours2;
						findNeighbours(id2[zzz], neighbours2, views_per_level);

						for (int xxx = 0; xxx < neighbours.size(); xxx++) {
							for (int yyy = 0; yyy < neighbours2.size(); yyy++) {
								if (neighbours.at(xxx) == neighbours2.at(yyy)) {
									sufficientDistance = false;
									break;
								}
							}
							if (!sufficientDistance) break;
						}
						if (!sufficientDistance) break;
					}

					if (sufficientDistance) {
						id1[kkk] = iii;
						id2[kkk] = jjj;
						scores[kkk] = matchingScores.at(iii).at(jjj);
					}
				}
			}

		}
	}

	for (int iii = 0; iii < num; iii++) {
		std::printf("%s << Match num (%d) is (%d) with score (%f)\n", __FUNCTION__, iii, id2[iii], scores[iii]);
	}

}

cv::Mat appendImage(const cv::Mat& im1, const cv::Mat& im2) {

	if (im1.type() != im2.type()) printf("%s << ERROR! Input types don't match.\n", __FUNCTION__);

	cv::Mat retMat = cv::Mat::zeros(max(im1.rows, im2.rows), im1.cols + im2.cols, im1.type());

	for (int iii = 0; iii < im1.rows; iii++) {
		for (int jjj = 0; jjj < im1.cols; jjj++) {
			if (im1.type() == CV_8UC3) {
				for (int kkk = 0; kkk < 3; kkk++) {
					retMat.at<cv::Vec3b>(iii,jjj)[kkk] = im1.at<cv::Vec3b>(iii,jjj)[kkk];
				}
			} else if (im1.type() == CV_16UC1) {
				retMat.at<unsigned short>(iii,jjj) = im1.at<unsigned short>(iii,jjj);
			} else if (im1.type() == CV_8UC1) {
				retMat.at<unsigned char>(iii,jjj) = im1.at<unsigned char>(iii,jjj);
			} else {
				printf("%s << ERROR! Unrecognized image type for appending.\n", __FUNCTION__);
			}
		}
	}

	for (int iii = 0; iii < im2.rows; iii++) {
		for (int jjj = 0; jjj < im2.cols; jjj++) {
			if (im1.type() == CV_8UC3) {
				for (int kkk = 0; kkk < 3; kkk++) {
					retMat.at<cv::Vec3b>(iii,jjj+im1.cols)[kkk] = im2.at<cv::Vec3b>(iii,jjj)[kkk];
				}
			} else if (im1.type() == CV_16UC1) {
				retMat.at<unsigned short>(iii,jjj+im1.cols) = im2.at<unsigned short>(iii,jjj);
			} else if (im1.type() == CV_8UC1) {
				retMat.at<unsigned char>(iii,jjj+im1.cols) = im2.at<unsigned char>(iii,jjj);
			} else {
				printf("%s << ERROR! Unrecognized image type for appending.\n", __FUNCTION__);
			}
		}
	}

	return retMat;
}

cv::Mat cropImageFixed(const cv::Mat& src, const cv::Mat& mask, int width, bool remask) {
	cv::Mat dst, temp, in_mask, mask8;

	(mask.rows == 0) ? src.copyTo(in_mask) : mask.copyTo(in_mask);
	(in_mask.type() == CV_8UC1) ? in_mask.copyTo(mask8) : cvtColor(in_mask, mask8, cv::COLOR_RGB2GRAY);

	(remask) ? src.copyTo(temp, mask8) : src.copyTo(temp);

	// Then cut temp
	int extents[4];
	findExtents(mask8, extents);

	int w_center = (extents[1]+extents[0])/2;
	int h_center = (extents[3]+extents[2])/2;
	int buff[4];
	buff[0] = width/2;
	buff[1] = width - buff[0];
	buff[2] = width/2;
	buff[3] = width - buff[2];

	int roi_pts[4];
	roi_pts[0] = w_center-buff[0];
	roi_pts[1] = h_center-buff[2];
	roi_pts[2] = width;
	roi_pts[3] = width;

	cv::Rect roi(roi_pts[0], roi_pts[1], roi_pts[2], roi_pts[3]);
	temp(roi).copyTo(dst);

	return dst;
}

cv::Mat cropImage(const cv::Mat& src, const cv::Mat& mask, int buffer, bool remask) {
	cv::Mat dst, temp, in_mask, mask8;

	(mask.rows == 0) ? src.copyTo(in_mask) : mask.copyTo(in_mask);
	(in_mask.type() == CV_8UC1) ? in_mask.copyTo(mask8) : cvtColor(in_mask, mask8, cv::COLOR_RGB2GRAY);

	(remask) ? src.copyTo(temp, mask8) : src.copyTo(temp);

	// Then cut temp
	int extents[4];
	findExtents(mask8, extents);

	int roi_pts[4];
	roi_pts[0] = max(0,extents[0]-buffer);
	roi_pts[1] = max(0,extents[2]-buffer);
	roi_pts[2] = min(temp.cols-1-roi_pts[0], extents[1]-extents[0]+buffer*2);
	roi_pts[3] = min(temp.rows-1-roi_pts[1], extents[3]-extents[2]+buffer*2);

	cv::Rect roi(roi_pts[0], roi_pts[1], roi_pts[2], roi_pts[3]);
	temp(roi).copyTo(dst);

	return dst;
}

cv::Mat extractObject(const cv::Mat& src, const cv::Mat& mask) {
	
	
	

	// Crop out object
	cv::Mat croppedImage = cropImage(src, mask);
	//imshow("croppedImage", croppedImage);
	//cv::waitKey();

	cv::Size idealSize(max(croppedImage.cols, croppedImage.rows), max(croppedImage.cols, croppedImage.rows));

	// Pad to make square
	cv::Mat squareImage = cv::Mat::zeros(idealSize, croppedImage.type());
	int missingRows = squareImage.rows - croppedImage.rows;
	int missingCols = squareImage.cols - croppedImage.cols;
	cv::Rect roi(missingCols/2, missingRows/2, croppedImage.cols, croppedImage.rows);
	croppedImage.copyTo(squareImage(roi));
	//imshow("squareImage", squareImage);
	//cv::waitKey();

	// cv::Mat retMat;
	// Shrink or blow up if necessary
	//while (squareImage.rows > 200) cv::resize(squareImage, squareImage, cv::Size(squareImage.cols/2, squareImage.rows/2), 0.0, 0.0, CV_INTER_NN);
	//while (squareImage.rows < 100) cv::resize(squareImage, squareImage, cv::Size(squareImage.cols*2, squareImage.rows*2), 0.0, 0.0, CV_INTER_NN);
	//imshow("squareImage", squareImage);
	//cv::waitKey();

	// Buff image
	//retMat = cv::Mat::zeros(idealSize, squareImage.type());
	//cv::Rect roi2((retMat.cols-squareImage.cols)/2, (retMat.rows-squareImage.rows)/2, squareImage.cols, squareImage.rows);
	//squareImage.copyTo(retMat(roi2));

	return squareImage;
}

cv::Mat buffImage(const cv::Mat& src, cv::Size sz) {
	
	cv::Mat src_cpy, dst;
	if ((sz.height < src.rows) || (sz.width < src.cols)) {
		double reductionFactor = max(double(src.rows)/double(sz.height), double(src.cols)/double(sz.width));
		cv::Size sz_tmp(int(double(src.rows)/reductionFactor), int(double(src.cols)/reductionFactor));
		resize(src, src_cpy, sz_tmp, 0.0, 0.0, CV_INTER_NN);
	} else src.copyTo(src_cpy);
	
	dst = cv::Mat::zeros(sz, src_cpy.type());

	int h_shift = (sz.width - src_cpy.cols)/2;
	int v_shift = (sz.height - src_cpy.rows)/2;

	for (int iii = 0; iii < src_cpy.rows; iii++) {
		for (int jjj = 0; jjj < src_cpy.cols; jjj++) {
			if (src_cpy.type() == CV_8UC3) {
				dst.at<cv::Vec3b>(iii+v_shift,jjj+h_shift)[0] = src_cpy.at<cv::Vec3b>(iii,jjj)[0];
				dst.at<cv::Vec3b>(iii+v_shift,jjj+h_shift)[1] = src_cpy.at<cv::Vec3b>(iii,jjj)[1];
				dst.at<cv::Vec3b>(iii+v_shift,jjj+h_shift)[2] = src_cpy.at<cv::Vec3b>(iii,jjj)[2];
			} else dst.at<unsigned char>(iii+v_shift,jjj+h_shift) = src_cpy.at<unsigned char>(iii,jjj);
			
		}
	}
	return dst;
}

void findNeighbours(const int& center, vector<int>& indices, int views_per_level) {

	indices.clear();

	int neighbour_scope = int(NEIGHBOUR_FRACTION*double(views_per_level));
	
	int currentSection = -1;
	while (center >= (currentSection+1)*views_per_level) currentSection++;

	for (int kkk = -(neighbour_scope-1)/2; kkk < (neighbour_scope+1)/2; kkk++) {
		int idx = center+kkk;

		int newSection = -1;
		while (idx >= (newSection+1)*views_per_level) newSection++;

		// If index overflowed negative
		if (newSection < currentSection) idx += views_per_level;

		// If index overflowed positive
		if (newSection > currentSection) idx -= views_per_level;

		indices.push_back(idx);
	}
}

void smoothMatchingScores(vector<vector<double> >& scores, int views_per_level) {
	
	vector<vector<double> > smoothed_scores;

	for (int iii = 0; iii < scores.size(); iii++) {
		vector<double> localScores;
		for (int jjj = 0; jjj < scores.at(iii).size(); jjj++) {
			double newScore = 0.0;

			int currentSection = -1;
			while (jjj >= (currentSection+1)*views_per_level) currentSection++;

			vector<int> neighbours;
			findNeighbours(jjj, neighbours, views_per_level);

			int contributingNeighbours = 0;
			for (int kkk = 0; kkk < neighbours.size(); kkk++) {
				if (scores.at(iii).at(neighbours.at(kkk)) < std::numeric_limits<double>::max()) {
					newScore += scores.at(iii).at(neighbours.at(kkk));
					contributingNeighbours++;
				}
			}

			(contributingNeighbours == 0) ? newScore = std::numeric_limits<double>::max() : newScore /= double(contributingNeighbours);
			localScores.push_back(newScore);
		}
		smoothed_scores.push_back(localScores);
	}

	scores = smoothed_scores;

}

double colourHistogramMatchScore(cv::Mat& test_hist, cv::Mat& lib_hist) {

	cv::Mat normalized_test_hist, normalized_lib_hist;
	normalizeHist(test_hist, normalized_test_hist);
	normalizeHist(lib_hist, normalized_lib_hist); // SHould be able to remove later

	double retVal = 0.0;

	for (int iii = 0; iii < test_hist.rows; iii++) {
		for (int jjj = 0; jjj < test_hist.cols; jjj++) {
			double a = normalized_test_hist.at<double>(iii,jjj);
			double b = normalized_lib_hist.at<double>(iii,jjj);

			retVal += abs(normalized_test_hist.at<double>(iii,jjj) - normalized_lib_hist.at<double>(iii,jjj));
		}
	}

	return retVal;
}

void normalizeHist(cv::Mat& src, cv::Mat& dst) {

	int sampleCount = 0;

	// Count total number of sampled pixels
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			sampleCount += int(src.at<double>(iii,jjj));
		}
	}

	// Find median intensity level (don't use this initially...)
	int medianLevel = -1, newCount = 0;
	for (int jjj = 0; jjj < src.cols; jjj++) {
		for (int iii = 0; iii < src.rows; iii++) {
			newCount += int(src.at<double>(iii,jjj));
			if (newCount > (sampleCount/2)) medianLevel = jjj;
		}
	}

	// Create new normalized histogram
	dst = cv::Mat::zeros(src.size(), src.type());
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			dst.at<double>(iii,jjj) = (src.at<double>(iii,jjj) / double(sampleCount));
		}
	}

}

void generateMatchScoreMatrix(vector<vector<double> > scores, cv::Mat& result, vector<int>& match_obj, vector<int>& match_img, int views_per_level, int obj_id, int img_id) {

	int verticalFactor = 1, horizontalFactor = 1;

	int buffer = 10;

	while (verticalFactor * scores.size() < 400) verticalFactor++;

	int maxSamples = 0;
	for (int iii = 0; iii < scores.size(); iii++) maxSamples = max(maxSamples, int(scores.at(iii).size()));

	//while (horizontalFactor * maxSamples < 800) horizontalFactor++;

	int buffer_count = (maxSamples/views_per_level)+1;

	result = cv::Mat::zeros(buffer*(int(scores.size())+1) + verticalFactor * int(scores.size()), buffer*buffer_count + horizontalFactor * maxSamples, CV_8UC3);

	double minScore = std::numeric_limits<double>::max();
	double maxScore = std::numeric_limits<double>::min();
	int minScore_obj = -1, minScore_img = -1;

	for (int iii = 0; iii < scores.size(); iii++) {
		for (int jjj = 0; jjj < scores.at(iii).size(); jjj++) {
			if (scores.at(iii).at(jjj) < minScore) {
				minScore = min(minScore, scores.at(iii).at(jjj));
				minScore_obj = iii;
				minScore_img = jjj;
			}
			if (scores.at(iii).at(jjj) < std::numeric_limits<double>::max()) {
				maxScore = max(maxScore, scores.at(iii).at(jjj));
			}
		}
	}

	// Fill out border
	for (int iii = 0; iii < result.rows; iii++) {
		for (int jjj = 0; jjj < result.cols; jjj++) {
			if ((iii < buffer) || (jjj < buffer) || (iii >= (result.rows-buffer)) || (jjj >= (result.cols-buffer))) {
				result.at<cv::Vec3b>(iii, jjj)[0] = 255;
				result.at<cv::Vec3b>(iii, jjj)[1] = 180;
				result.at<cv::Vec3b>(iii, jjj)[2] = 180;
			} else if ((jjj % (buffer+horizontalFactor*views_per_level)) < buffer) {
				result.at<cv::Vec3b>(iii, jjj)[0] = 255;
				result.at<cv::Vec3b>(iii, jjj)[1] = 180;
				result.at<cv::Vec3b>(iii, jjj)[2] = 180;
			} else if ((iii % (buffer+verticalFactor)) < buffer) {
				result.at<cv::Vec3b>(iii, jjj)[0] = 255;
				result.at<cv::Vec3b>(iii, jjj)[1] = 180;
				result.at<cv::Vec3b>(iii, jjj)[2] = 180;
			}
		}
	}

	// Highlight neighbourhood of ground truth
	vector<int> neighbours;
	findNeighbours(img_id, neighbours, views_per_level);
	int image_segment = img_id / views_per_level;
	for (int iii = 0; iii < result.rows; iii++) {
		for (int jjj = 0; jjj < result.cols; jjj++) {
			if ((iii < buffer) || (jjj < buffer)) {

				if (((iii-buffer*(obj_id+1)) >= verticalFactor*obj_id) && ((iii-buffer*(obj_id+1)) < verticalFactor*(obj_id+1))) {
					result.at<cv::Vec3b>(iii, jjj)[1] = 0;
					result.at<cv::Vec3b>(iii, jjj)[2] = 0;
				}

				if (((jjj-buffer*(image_segment+1)) >= horizontalFactor*img_id) && ((jjj-buffer*(image_segment+1)) < horizontalFactor*(img_id+1))) {
					for (int kkk = 0; kkk < neighbours.size(); kkk++) {
						for (int bbb = 0; bbb < horizontalFactor; bbb++) {
							result.at<cv::Vec3b>(iii, buffer*(image_segment+1)+horizontalFactor*neighbours.at(kkk)+bbb)[1] = 0;
							result.at<cv::Vec3b>(iii, buffer*(image_segment+1)+horizontalFactor*neighbours.at(kkk)+bbb)[2] = 0;
						}
					}
				}
			}
		}
	}

	// Fill in scores
	for (int iii = 0; iii < scores.size(); iii++) {
		for (int jjj = 0; jjj < scores.at(iii).size(); jjj++) {
			int local_image_segment = jjj / views_per_level;
			for (int aaa = 0; aaa < verticalFactor; aaa++) {
				for (int bbb = 0; bbb < horizontalFactor; bbb++) {
					for (int kkk = 0; kkk < 3; kkk++) {
						double score = scores.at(iii).at(jjj);
						double val = min(255.0, max(0.0, (score - minScore)/(maxScore - minScore)));
						if (score == std::numeric_limits<double>::max()) {
							result.at<cv::Vec3b>(buffer*(1+iii)+iii*verticalFactor+aaa, buffer*(local_image_segment+1)+jjj*horizontalFactor+bbb)[kkk] = (kkk == 2) ? 255 : 0;
						}  else result.at<cv::Vec3b>(buffer*(1+iii)+iii*verticalFactor+aaa, buffer*(local_image_segment+1)+jjj*horizontalFactor+bbb)[kkk] = (unsigned char)(255.0 - 255.0*pow(val, 0.5));
					}
				}
			}
		}
	}
	
	if (minScore_obj == -1) {
		std::printf("%s << ERROR! No matches were achieved at all!\n", __FUNCTION__);
		return;
	}

	// Highlight best score results

	for (int xxx = 1; xxx < match_img.size(); xxx++) {
		vector<int> matched_neighbours;
		findNeighbours(match_img[xxx], matched_neighbours, views_per_level);
		for (int zzz = 0; zzz < matched_neighbours.size(); zzz++) {
			for (int aaa = 0; aaa < verticalFactor; aaa++) {
				for (int bbb = 0; bbb < horizontalFactor; bbb++) {
					for (int kkk = 0; kkk < 3; kkk++) {
						result.at<cv::Vec3b>(buffer*(1+match_obj[xxx])+match_obj[xxx]*verticalFactor+aaa, buffer*(1+(match_img[xxx]/views_per_level))+matched_neighbours.at(zzz)*horizontalFactor+bbb)[kkk] = (kkk == 1) ? 90 : 0;
					}
				}
			}
		}

	}

	vector<int> matched_neighbours;
	findNeighbours(minScore_img, matched_neighbours, views_per_level);
	for (int zzz = 0; zzz < matched_neighbours.size(); zzz++) {
		for (int aaa = 0; aaa < verticalFactor; aaa++) {
			for (int bbb = 0; bbb < horizontalFactor; bbb++) {
				for (int kkk = 0; kkk < 3; kkk++) {
					result.at<cv::Vec3b>(buffer*(1+minScore_obj)+minScore_obj*verticalFactor+aaa, buffer*(1+(minScore_img/views_per_level))+matched_neighbours.at(zzz)*horizontalFactor+bbb)[kkk] = (kkk == 1) ? 180 : 0;
				}
			}
		}
	}
}

bool compareByResponse(cv::KeyPoint a, cv::KeyPoint b) {
    return a.response < b.response;
}

void determineObjectEdgePixels(const cv::Mat& mask, vector<cv::Point>& pixelLocs) {

	cv::Mat edge = cv::Mat::zeros(mask.size(), CV_8UC1);

	for (int iii = 0; iii < mask.rows; iii++) {
		for (int jjj = 0; jjj < mask.cols; jjj++) {
			if (mask.at<unsigned char>(iii,jjj) == 0) continue;

			bool fullyChecked = false;
			for (int aaa = std::max<int>(0, iii-1); aaa < std::min<int>(mask.rows, iii+2); aaa++) {
				for (int bbb = std::max<int>(0, jjj-1); bbb < std::min<int>(mask.cols, jjj+2); bbb++) {
					if (mask.at<unsigned char>(aaa, bbb) == 0) {
						pixelLocs.push_back(cv::Point(jjj,iii));
						edge.at<unsigned char>(iii,jjj) = 255;
						fullyChecked = true;
						break;
					}
				}
				if (fullyChecked) break;
			}
		}
	}

	/*
	imshow("edge", edge);
	cv::waitKey(1000);
	*/

}

bool doesFeatureIntersect(const vector<cv::Point>& pixelLocs, const cv::KeyPoint& kp) {

	for (int iii = 0; iii < pixelLocs.size(); iii++) {
		if (pow(pow(pixelLocs.at(iii).x - kp.pt.x, 2.0) + pow(pixelLocs.at(iii).y - kp.pt.y, 2.0), 0.5) < (kp.size/2.0)) return true;
	}

	return false;
}

void histogramNormalize(const cv::Mat& src, cv::Mat& dst, const cv::Mat &mask, float percent_lo, float percent_hi) {

	assert(src.channels() == 3);
    assert(percent_lo > 0 && percent_lo < 100 && percent_hi > 0 && percent_hi < 100);
 
    vector<cv::Mat> tmpsplit; 
	split(src,tmpsplit);
	cv::Mat mask_flat;
	cv::Mat mask_single;

	(mask.type() != CV_8UC1) ? cvtColor(mask, mask_single, cv::COLOR_RGB2GRAY) : mask.copyTo(mask_single);
	mask_single.reshape(1,1).copyTo(mask_flat);
	//std::printf("%s << mask_single.type() = (%d) vs (%d, %d, %d, %d, %d)\n", __FUNCTION__, mask_single.type(), CV_8UC1, CV_8UC3, CV_16UC1, CV_8SC1, CV_8UC4);

    for(int i=0;i<3;i++) {
        //find the low and high precentile values (based on the input percentile)
        cv::Mat flat; 
		tmpsplit[i].reshape(1,1).copyTo(flat);
		
		// Need to get rid of the pixels that don't align with mask!
		//cv::imshow("mask_single", mask_single);
		//cv::waitKey(0);
		int validPixels = cv::countNonZero(mask_single);
		cv::Mat flat_reduced = cv::Mat::zeros(1, validPixels, flat.type());
		
		int index = 0;
		for (int iii = 0; iii < mask_flat.cols; iii++) {
			if (mask_flat.at<uchar>(iii) > 0) {
				flat_reduced.at<uchar>(index) = flat.at<uchar>(iii);
				index++;
			}
		}

        cv::sort(flat_reduced,flat_reduced,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        int lowval = flat_reduced.at<uchar>(cvFloor(((float)flat_reduced.cols) * percent_lo/100.0));
        int highval = flat_reduced.at<uchar>(cvCeil(((float)flat_reduced.cols) * (1.0 - percent_hi/100.0)));
        //cout << lowval << " " << highval << endl;
        
        //saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval,tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval,tmpsplit[i] > highval);
        
        //scale the channel
        normalize(tmpsplit[i],tmpsplit[i],0,255,cv::NORM_MINMAX);
    }
    merge(tmpsplit,dst);

	/*
	cv::imshow("normalization", src);
	cv::waitKey(0);
	cv::imshow("normalization", dst);
	cv::waitKey(0);
	*/

}

void addToDebugImage(const cv::Mat& src, cv::Mat& dst, int row, int col) {
	if (dst.rows == 0) dst = cv::Mat::zeros(4*src.rows, 2*src.cols, CV_8UC3);

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			for (int kkk = 0; kkk < 3; kkk++) {
				dst.at<cv::Vec3b>(row*src.rows+iii,col*src.cols+jjj)[kkk] = src.at<cv::Vec3b>(iii,jjj)[kkk];
			}
		}
	}

}

int countBlobsInMask(const cv::Mat& src) {

	cv::Mat flat_src;
	(src.type() != CV_8UC1) ? cvtColor(src, flat_src, cv::COLOR_RGB2GRAY) : src.copyTo(flat_src);

	vector<unsigned char> uniqueLevels;
	
	for (int iii = 0; iii < flat_src.rows; iii++) {
		for (int jjj = 0; jjj < flat_src.cols; jjj++) {
			bool isUnique = true;
			for (int kkk = 0; kkk < uniqueLevels.size(); kkk++) {
				if (flat_src.at<unsigned char>(iii,jjj) == uniqueLevels.at(kkk)) {
					isUnique = false;
					break;
				}
			}
			if (isUnique) {
				uniqueLevels.push_back(flat_src.at<unsigned char>(iii,jjj));
			}
		}
	}

	return int(uniqueLevels.size()-1);
}

string get_address(string address_file, string default_address) {
	char address[256];
	sprintf(address, "%s/params/%s", _SOURCE_DIRECTORY_, address_file.c_str());
	ifstream addressFile;
	addressFile.open (address);
	string addr;
	std::getline(addressFile, addr);
	addressFile.close();
	
	if (addr.length() == 0) {
		addr = string(_SOURCE_DIRECTORY_) + "/" + default_address;
		//std::printf("Using default directory (%s)\n", addr.c_str());
	} //else std::printf("Found directory (%s)\n", addr.c_str());
	return addr;
}

void drawHistogram(const cv::Mat& src, cv::Mat& dst) {

	int hist_w = 512, hist_h = 400;
	int *bins_w;
	bins_w = new int(src.dims);

	for (int iii = 0; iii < src.dims; iii++) {
		bins_w[iii] = cvRound( (double) hist_w/(src.size[iii]-1) );
		//std::printf("%s << bins_w[%d] = (%d) from (%d) and (%d)\n", __FUNCTION__, iii, bins_w[iii], hist_w, sizes[count]);
	}

	dst = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	vector<cv::Mat> channelMats;
	for (int iii = 0; iii < src.dims; iii++) {
		int length = src.size[iii];
		cv::Mat blankMat = cv::Mat::zeros(1, length, CV_32FC1);
		channelMats.push_back(blankMat);
	}

	// Grouping
	for( int i = 0; i < src.size[0]; i++ ) {
		if (src.dims > 1) {
			for( int j = 0; j < src.size[1]; j++ ) {
				if (src.dims > 2) {
					for (int k = 0; k < src.size[2]; k++ ) {
						channelMats.at(0).at<float>(i) += src.at<float>(i, j, k);
						channelMats.at(1).at<float>(j) += src.at<float>(i, j, k);
						channelMats.at(2).at<float>(k) += src.at<float>(i, j, k);
						//std::printf("%s << Adding: (%f)\n", __FUNCTION__, src.at<float>(i, j, k));
					}
				} else {
					channelMats.at(0).at<float>(i) += src.at<float>(i, j);
					channelMats.at(1).at<float>(j) += src.at<float>(i, j);
				}
			}
		} else {
			channelMats.at(0).at<float>(i) += src.at<float>(i);
		}
	}

	vector<cv::Scalar> colors(3);
	colors[0] = cv::Scalar( 255, 0, 0);
	colors[1] = cv::Scalar( 0, 255, 0);
	colors[2] = cv::Scalar( 0, 0, 255);

	// Determine appropriate maximum vertical scaling value
	float maxVal = 0.25;

	for (int iii = 0; iii < src.dims; iii++) {
		// Normalize for display
		for( int i = 0; i < src.size[iii]; i++ ) channelMats.at(iii).at<float>(i) /= maxVal;
		for( int i = 1; i < src.size[iii]; i++ ) {

			//std::printf("%s << y-val = (%f) with (%f)\n", __FUNCTION__, float(hist_h)*channelMats.at(iii).at<float>(i-1), float(hist_h));
			//std::printf("%s << iii = (%d), i = (%d), x1 = (%d) and x2 = (%d), bins_w[%d] = (%d)\n", __FUNCTION__, iii, i, bins_w[iii]*(i-1), bins_w[iii]*(i), iii, bins_w[iii]);

			line(dst, cv::Point( bins_w[iii]*(i-1), hist_h - cvRound(float(hist_h)*channelMats.at(iii).at<float>(i-1)) ) ,
							cv::Point( bins_w[iii]*(i), hist_h - cvRound(float(hist_h)*channelMats.at(iii).at<float>(i)) ),
							colors[iii], 2, 8, 0  );
		}
	}
}

void padNames(vector<string>& names, bool is_mask) {

	for (int iii = 0; iii < names.size(); iii++) {
		size_t found = names.at(iii).find_last_of("_");

		if (found != string::npos) {
			if (is_mask) {
				while ((names.at(iii).size() - found) < 13) names.at(iii).insert(names.at(iii).begin()+found+1, 1, '0'); 
			} else {
				while ((names.at(iii).size() - found) < 8) names.at(iii).insert(names.at(iii).begin()+found+1, 1, '0'); 
			}
		}
	}

}

void unpadNames(vector<string>& names) {

	for (int iii = 0; iii < names.size(); iii++) {

		while (names.at(iii).find("_0") != string::npos) {
			size_t index = 0;
			index = names.at(iii).find("_0");
			names.at(iii).replace(index, 2, "_");
		}

		size_t index = 0;
		index = names.at(iii).find("_.");
		if (index != string::npos) names.at(iii).replace(index, 2, "_0.");

	}
}

void findExtents(const cv::Mat& mask, int *extents) {
	
	if (extents == NULL) extents = new int[4];
	extents[0] = mask.cols; // min_x
	extents[1] = 0; // max_x
	extents[2] = mask.rows; // min_y
	extents[3] = 0; // max_y

	for (int iii = 0; iii < mask.rows; iii++) {
		for (int jjj = 0; jjj < mask.cols; jjj++) {
			if (mask.at<unsigned char>(iii, jjj) > 0) {
				extents[0] = min(extents[0], jjj);
				extents[1] = max(extents[1], jjj);
				extents[2] = min(extents[2], iii);
				extents[3] = max(extents[3], iii);
			}
		}
	}
}

void cleanMask(const cv::Mat& src, cv::Mat& dst) {

	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*DEFAULT_EROSION_SIZE + 1, 2*DEFAULT_EROSION_SIZE+1 ), cv::Point( DEFAULT_EROSION_SIZE, DEFAULT_EROSION_SIZE ) );

	//erosionMask = 255 - erosionMask;

	cv::Mat tmp;

	//Opening morphological operation - erode then dilate. Clears noise in image.
	cv::erode( src, tmp, element );
	cv::dilate(tmp, dst, element);
}

double calculateConfidence(const double ratio, const vector<double>& interpolationPoints) {

	double workingRatio = min(ratio, 1/ratio);

	double retVal = 0.0;

	if (interpolationPoints.size() == 0) return 0.0;

	if (workingRatio == 0.0) {
		retVal = interpolationPoints.at(0);
	} else if (workingRatio == 1.0) {
		retVal = interpolationPoints.at(interpolationPoints.size()-1);
	} else {
		double percMarker = 0.0;
		for (int idx = 0; idx < interpolationPoints.size()-1; idx++) {
			percMarker = double(idx+1.0)/double(interpolationPoints.size()-1);
			if (workingRatio <= percMarker) {
				retVal = ( (percMarker-workingRatio)*interpolationPoints.at(idx) + (1-(percMarker-workingRatio))*interpolationPoints.at(idx+1) );
				break;
			}
		}
	}

	return (ratio <= 1.0) ? retVal : 1-retVal;

}

double viewCompatibilityScore(const vector<double>& scores, int correct_index, int views_per_level) {

	int min_index = -1;
	double min_score = std::numeric_limits<double>::max();

	for (int iii = 0; iii < scores.size(); iii++) {
		if (scores.at(iii) < min_score) {
			min_score = scores.at(iii);
			min_index = iii;
		}
	}

	if (min_index == correct_index) return 1.0;
	return 0.0;

}

void showViewMatchingResult(const vector<double>& scores, int views_per_level) {
	
	string windowName = "viewMatchingResult";

	int levels = int(scores.size()) / views_per_level;

	cv::Mat resultMatrix = cv::Mat::zeros(levels, views_per_level, CV_64FC1);

	for (int iii = 0; iii < resultMatrix.rows; iii++) {
		for (int jjj = 0; jjj < resultMatrix.cols; jjj++) {
			resultMatrix.at<double>(iii,jjj) = scores.at(iii*views_per_level+jjj);
		}
	}

	expandAndDisplay(resultMatrix, windowName);

}

double calculateViewError(int level_index_1, int frame_index_1, int level_index_2, int frame_index_2, int level_count, int frame_count) {
	int level_diff = abs(level_index_1-level_index_2);
	// if (double(level_diff) > (double(level_count/2.0))) level_diff = level_count - level_diff; // Levels DO NOT loop around (at least at the moment..)
	
	int frame_diff = abs(frame_index_1-frame_index_2);
	if (double(frame_diff) > (double(180.0))) frame_diff = 180 - frame_diff;

	double error = pow(pow((double(level_diff) / double(max(1,level_count-1))), 2.0) + pow((double(frame_diff) / double(180.0)), 2.0), 0.5); 

	return error;
}

cv::Mat expandAndDisplay(const cv::Mat& src, string windowName, double max_val) {

	if (src.rows == 0) return cv::Mat();

	int multFactor_rows = 1;
	while ((multFactor_rows * src.rows) < 500) multFactor_rows++;

	int multFactor_cols = 1;
	while ((multFactor_cols * src.cols) < 500) multFactor_cols++;

	cv::Size sz(src.rows*multFactor_rows, src.cols*multFactor_cols);

	cv::Mat resizedMat, dispMat_grey, dispMat_col;
	resize(src, resizedMat, sz, 0.0, 0.0, CV_INTER_NN);

	double min, max;
	cv::minMaxLoc(src, &min, &max);

	if (max_val != 0.0) max = max_val;
	resizedMat.convertTo(dispMat_grey, CV_8UC1, 255.0/max, 0);
	cv::cvtColor(dispMat_grey, dispMat_col, cv::COLOR_GRAY2RGB);

	for (int iii = 0; iii < resizedMat.rows; iii++) {
		for (int jjj = 0; jjj < resizedMat.cols; jjj++) {
			if (resizedMat.at<double>(iii,jjj) == -1.0) { 
				dispMat_col.at<cv::Vec3b>(iii,jjj)[0] = 0;
				dispMat_col.at<cv::Vec3b>(iii,jjj)[1] = 0;
				dispMat_col.at<cv::Vec3b>(iii,jjj)[2] = 255;
			} else if (resizedMat.at<double>(iii,jjj) == -2.0) { 
				dispMat_col.at<cv::Vec3b>(iii,jjj)[0] = 255;
				dispMat_col.at<cv::Vec3b>(iii,jjj)[1] = 0;
				dispMat_col.at<cv::Vec3b>(iii,jjj)[2] = 0;
			}
		}
	}
			
	cv::imshow(windowName, dispMat_col);

	return dispMat_grey;
}

double getPercentileValue(const vector<double>& vals, double percentile) {

	percentile = min(max(percentile, 0.0), 100.0);

	vector<double> sortedVec;
	sortedVec = vals;
	sort(sortedVec.begin(), sortedVec.end());

	return sortedVec.at(float2int(float(percentile) * float(sortedVec.size()-1.0)));

}

double calculateScoreFromDMatchVector(const vector<vector<cv::DMatch> >& matches1to2) {
	double retVal = 0.0;

	if (matches1to2.size() < MIN_MATCHES) return std::numeric_limits<double>::max();
	for (int iii = 0; iii < matches1to2.size(); iii++) {
		retVal += matches1to2.at(iii).at(0).distance;
	}

	retVal /= pow(double(matches1to2.size()), 2.0);

	if (retVal < 0.1) printf("%s << retVal = 0\n", __FUNCTION__);
	return retVal;
}



bool readXfTransformation(const std::string& src, cv::Mat& dst) {
	std::ifstream ifs(src.c_str());

	if (!ifs.is_open()) return false;

	dst = cv::Mat::zeros(4, 4, CV_64FC1);
	for (int aaa = 0; aaa < 4; aaa++) {
		for (int bbb = 0; bbb < 4; bbb++) ifs >> dst.at<double>(aaa,bbb);
	}

	ifs.close();
	return true;
}
