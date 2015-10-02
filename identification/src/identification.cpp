#include "identification.h"

#undef max
#undef min

#ifdef _IS_LINUX_
objectIdentifier::objectIdentifier(ros::NodeHandle& nh, string library_address, string params_directory)
{
	init();

    loadLibrary(library_address);
    ROS_INFO("Identification YML Library loaded.");
    
    loadConfidencePoints(params_directory);
    ROS_INFO("Identification Probabilities loaded.");
    
    service = nh.advertiseService("identify", &objectIdentifier::identification, this);
    ROS_INFO("Identification Server started.");

}

bool objectIdentifier::identification(identification::SRV_Identification::Request  &req,
         identification::SRV_Identification::Response &res)
{

	bool debugMode = false;

    // Process data from REQ
    string rgb_address = req.folder + "/rgbImage.png";
    string mask_address = req.folder + "/maskImage.png";
	string depth_address = req.folder + "/depthImage.png";
    string param_address = req.folder + "/param.yml";
    vector<string> realObjects;
    for (int iii = 0; iii < req.object_count; iii++) realObjects.push_back(req.search_objects[iii]);

    // Read in images
    cv::Mat rgbImage = cv::imread(rgb_address);
    cv::Mat maskImage = cv::imread(mask_address);
	cv::Mat depthImage = cv::imread(depth_address, CV_LOAD_IMAGE_ANYDEPTH);

	if (maskImage.rows == 0) {
		std::printf("%s << WARNING. Provided mask is empty, so attempting to synthesize one.\n", __FUNCTION__);
		maskImage = generateMask(rgbImage, req.object_count);
		cv::imwrite(mask_address, maskImage);
	}
    supplyLatestImage(rgbImage, maskImage, depthImage);
    
    // Read in ROI
    cv::Rect roiRect;
    cv::FileStorage fs(param_address, cv::FileStorage::READ);
	fs["ROI region"] >> roiRect;
	fs.release();

    // Prepare variables for algorithm output
    int blob_index;
    double confidence, preConfidence;
    vector<vector<double> > viewRotations;
	vector<string> versionNames;

    // Perform identification
    identifyObject(realObjects, 0, DEFAULT_RETURNED_VIEWS, blob_index, confidence, viewRotations, versionNames, preConfidence, debugMode);

    // Prepare output message
    res.blob_index = blob_index;
    res.confidence = confidence;
    res.pre_confidence = preConfidence;

    for (int iii = 0; iii < viewRotations.size(); iii++) {
		res.versions.push_back(versionNames.at(iii));
        for (int jjj = 0; jjj < 9; jjj++) {
            res.rotations.push_back(viewRotations.at(iii).at(jjj));
        }
    }
    
    // Debugging
    cv::Mat resultImage = generateDisplayImage(roiRect);
    imshow("capture-and-identification", resultImage);
	cv::waitKey(30);
	
    std::string result_address = req.folder + "/resultImage.png";
    cv::imwrite(result_address, resultImage);

    return true;
}
#endif

void objectIdentifier::init() {

	srand (((unsigned int)time(NULL)));

	counter = 0;

	for (int iii = 0; iii < MATCHING_METHODS_COUNT; iii++) {
		stringstream ss;
		ss << "identity_confidence_pts_" << iii << ".txt";
		confidencePointsFiles[iii] = ss.str();
		stringstream ss2;
		ss2 << "view_ratio_pts_" << iii << ".txt";
		viewRatioPointsFiles[iii] = ss2.str();

		idPriorProb3[iii] = NULL; 
		idSuccess3[iii] = NULL;
		idFailure3[iii] = NULL;
	}

	idPriorProb3[MATCHING_METHODS_COUNT] = NULL; 
	idSuccess3[MATCHING_METHODS_COUNT] = NULL;
	idFailure3[MATCHING_METHODS_COUNT] = NULL;

	amm = std::vector<int>(active_matching_methods,active_matching_methods + sizeof(active_matching_methods)/sizeof(active_matching_methods[0]));

	if (amm.at(0) == -1) {
		amm.clear();
		for (int iii = 0; iii < MATCHING_METHODS_COUNT; iii++) amm.push_back(iii);
	} 
}

void imset::release() {
	inputImage.release(); 
	inputMask.release(); 
	inputDepth.release();
	grayIm.release(); 
	grayColIm.release(); 

	for (int iii = 0; iii < blobs.size(); iii++) blobs[iii].featuresIm.release();
}

void imset::setDescName(int idx) {
	while (blobs.size() <= idx) {
		blob b;
		blobs.push_back(b);
	}
	size_t index = 0;
	index = rgbName.find(".", index);
	blobs[idx].viewName = rgbName.substr(0, index);
}

void objectLibrary::generate(bool debug) {

	prepareLibraryDirectories();

	// Loop that does previous stuff
	for (int iii = 0; iii < images->size(); iii++) {
		for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {
			loadImage(iii,jjj);
			int non_zero_pix = cv::countNonZero(images->at(iii).at(jjj).inputMask);
			if (non_zero_pix < MIN_PIXELS_FOR_VALID_MASK) continue;
			images->at(iii).at(jjj).describe(debug);
			images->at(iii).at(jjj).release();
			saveFeatureLibraryComponent(iii,jjj);
		}
	}
}

void objectIdentifier::generateLibrary(string src, string dst, bool debug) {
	lib.setImageDirectory(src);
	lib.setYMLDirectory(dst);
	lib.traverseDirectory();
	lib.generate(debug);
}

void imageDirectory::loadImageLocations() {
	for (int iii = 0; iii < images->size(); iii++) {
		for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {
			images->at(iii).at(jjj).rgbName = images->at(iii).at(jjj).blobs[0].viewName + ".png";
			images->at(iii).at(jjj).maskName = images->at(iii).at(jjj).blobs[0].viewName + ".png";
			images->at(iii).at(jjj).depthName = images->at(iii).at(jjj).blobs[0].viewName + ".png";
		}
	}
}

bool imageDirectory::traverseDirectory() {
	
	images = new vector<vector<imset> >();

	// First determine directory format
	vector<string> testFolders;
	if (!getContents(image_directory, testFolders)) return false;
	directoryFormat = SINGLE_OBJECT_FORMAT;
	for (int iii = 0; iii < testFolders.size(); iii++) {
		if (testFolders.at(iii) == "rgb") {
			directoryFormat = MULTI_OBJECT_FORMAT; 
			break;
		}
	}

	if (directoryFormat == SINGLE_OBJECT_FORMAT) {
		if (objectFolders.size() == 0) {
			if (!getContents(image_directory, objectFolders)) return false;
			filterContents(objectFolders);
		}
		
		if (acceptableObjectIndices.size() != 0) {
			if (acceptableObjectIndices.at(0) != -1) {
				vector<string> reducedFolders;
				for (int iii = 0; iii < acceptableObjectIndices.size(); iii++) {
					string acceptableFolder;
					acceptableFolder = objectFolders.at(acceptableObjectIndices.at(iii));
					reducedFolders.push_back(acceptableFolder);
				}
				objectFolders = reducedFolders;
			}
		}

		for (int iii = 0; iii < objectFolders.size(); iii++) {

			std::vector<std::string> rgbNamesLocal;
		
			string inputDirectory = image_directory + "/" + objectFolders.at(iii) + "/rgbd/rgb";

			if (!getContents(inputDirectory, rgbNamesLocal)) return false;
			filterContents(rgbNamesLocal, allowableElevations);

			padNames(rgbNamesLocal);
			sort(rgbNamesLocal.begin(), rgbNamesLocal.end());
			unpadNames(rgbNamesLocal);
		
			vector<imset> localObjectVec;
			for (int jjj = 0; jjj < rgbNamesLocal.size(); jjj++) {
				imset localObject;
				localObject.rgbName = rgbNamesLocal.at(jjj);
				localObject.setDescName();
				localObject.blobs[0].objectName = objectFolders.at(iii);
				localObjectVec.push_back(localObject);
			}
		
			// And Load mask images
			std::vector<std::string> maskNamesLocal;
		
			inputDirectory = image_directory + "/" + objectFolders.at(iii) + "/rgbd/mask";

			if (!getContents(inputDirectory, maskNamesLocal)) return false;
			filterContents(maskNamesLocal, allowableElevations, true);

			padNames(maskNamesLocal, true);
			sort(maskNamesLocal.begin(), maskNamesLocal.end());
			unpadNames(maskNamesLocal);

			for (int jjj = 0; jjj < maskNamesLocal.size(); jjj++) {
				size_t index = 0;
				index = maskNamesLocal.at(jjj).find(".mask", index);
				if (index != string::npos) maskNamesLocal.at(jjj).replace(index, 5, "_mask");
			}

			if (maskNamesLocal.size() != localObjectVec.size()) {
				std::printf("%s << ERROR! Number of existing local images for <%s> doesn't match number of available mask names.\n", __FUNCTION__, objectFolders.at(iii).c_str());
				return false;
			}

			for (int jjj = 0; jjj < maskNamesLocal.size(); jjj++) localObjectVec.at(jjj).maskName = maskNamesLocal.at(jjj);

			// And load depth images
			std::vector<std::string> depthNamesLocal;
		
			inputDirectory = image_directory + "/" + objectFolders.at(iii) + "/rgbd/depth";

			if (!getContents(inputDirectory, depthNamesLocal)) return false;
			filterContents(depthNamesLocal, allowableElevations, true);

			padNames(depthNamesLocal, true);
			sort(depthNamesLocal.begin(), depthNamesLocal.end());
			unpadNames(depthNamesLocal);

			if (depthNamesLocal.size() != localObjectVec.size()) {
				std::printf("%s << ERROR! Number of existing local images for <%s> doesn't match number of available depth names.\n", __FUNCTION__, objectFolders.at(iii).c_str());
				return false;
			}

			for (int jjj = 0; jjj < depthNamesLocal.size(); jjj++) localObjectVec.at(jjj).depthName = depthNamesLocal.at(jjj);

			// For each RGB image, find if it has a rotation, and only if so, load it (otherwise load identity?)
			for (int jjj = 0; jjj < rgbNamesLocal.size(); jjj++) {
				string rotationName = rgbNamesLocal.at(jjj);

				size_t index = 0;
				index = rotationName.find(".", index);
				if (index != string::npos) {
					rotationName.replace(index+1, 3, "aln");
					index = 0;
				}

				string filename = image_directory + "/" + objectFolders.at(iii) + "/rgbd/rotation/" + rotationName;

				ifstream rotationsFile;
				rotationsFile.open(filename.c_str());
				
				if (rotationsFile.is_open()) {
					vector<double> rotation;
					//readRotationFromTxtFile(rotationsFile, rotation);
					if (readRotationFromAlnFile(rotationsFile, rotation)) {
						localObjectVec.at(jjj).blobs[0].rotation = rotation;
					}
					rotationsFile.close();
				}

			}

			images->push_back(localObjectVec);
		}

		std::printf("%s << (%d) objects found, total of (%d) views\n", __FUNCTION__, int(images->size()), int(countViews(images)));

	} else if (directoryFormat == MULTI_OBJECT_FORMAT) { 
		// Should load as if they are just a series of multi-object images with information about which objects are in each image, rather than directories labelled by objects

		// New alternative traversal that handles non-library arrangements (depends on input mode) sets a single object, but loads all relevant info for each image

		objectFolders.clear();
		objectFolders.push_back(".");

		vector<imset> dummyImageSetVec;
		images->push_back(dummyImageSetVec);
		
		// Load RGB images
		{
			std::vector<std::string> rgbNamesLocal;
			string inputDirectory = image_directory + "/rgb";

			if (!getContents(inputDirectory, rgbNamesLocal)) return false;

			//padNames(rgbNamesLocal);
			sort(rgbNamesLocal.begin(), rgbNamesLocal.end());
			//unpadNames(rgbNamesLocal);
		
			
			for (int jjj = 0; jjj < rgbNamesLocal.size(); jjj++) {
				imset localImageSet;
				localImageSet.rgbName = rgbNamesLocal.at(jjj);
				localImageSet.setDescName();
				images->at(0).push_back(localImageSet);
			}
		}

		// And load labels..
		{
			std::vector<std::string> labelNamesLocal;
		
			string inputDirectory = image_directory + "/labels";

			if (!getContents(inputDirectory, labelNamesLocal)) return false;
			
			//padNames(labelNamesLocal, true);
			sort(labelNamesLocal.begin(), labelNamesLocal.end());
			//unpadNames(labelNamesLocal);

			
			if (labelNamesLocal.size() != images->at(0).size()) {
				std::printf("%s << ERROR! Number of existing local images doesn't match number of available mask names.\n", __FUNCTION__);
				return false;
			}

			for (int iii = 0; iii < labelNamesLocal.size(); iii++) {

				ifstream labelsFile;
				string filename = inputDirectory + "/" + labelNamesLocal.at(iii);
				labelsFile.open(filename.c_str());
				string current_line, label_name, view_name;

				int local_blob_count = 0;
				while (std::getline(labelsFile, current_line)) {
					size_t index = 0;

					// If space in name, get index
					index = current_line.find(" ", index);
					if (index != string::npos) {
						label_name = current_line.substr(0, index);
						view_name = current_line.substr(index+1, current_line.length()-(index+1));
					} else {
						label_name = current_line;
						index = 0;
						index = label_name.find("\r", index);
						while (index != string::npos) {
							label_name.replace(index, 1, "");
							index = 0;
							index = label_name.find("\r", index);
						}
						if (label_name == "") break;
					}

					if (local_blob_count >= images->at(0).at(iii).blobs.size()) {
						blob b;
						images->at(0).at(iii).blobs.push_back(b);
					}
					
					images->at(0).at(iii).blobs[local_blob_count].objectName = label_name;
					images->at(0).at(iii).blobs[local_blob_count].viewName = view_name;
					local_blob_count++;
				}

				labelsFile.close();
			}
		}

		// And Load mask images
		{
			std::vector<std::string> maskNamesLocal;
		
			string inputDirectory = image_directory + "/mask";

			if (!getContents(inputDirectory, maskNamesLocal)) {
				std::printf("%s << No masks found! Skipping, and allowing system to synthesise them later..\n", __FUNCTION__);
			} else {

				//padNames(maskNamesLocal, true);
				sort(maskNamesLocal.begin(), maskNamesLocal.end());
				//unpadNames(maskNamesLocal);

				for (int iii = 0; iii < maskNamesLocal.size(); iii++) {
					size_t index = 0;
					index = maskNamesLocal.at(iii).find(".mask", index);
					if (index != string::npos) maskNamesLocal.at(iii).replace(index, 5, "_mask");
				}

				if (maskNamesLocal.size() != images->at(0).size()) {
					std::printf("%s << ERROR! Number of existing local images doesn't match number of available mask names.\n", __FUNCTION__);
					return false;
				}

				for (int iii = 0; iii < maskNamesLocal.size(); iii++) images->at(0).at(iii).maskName = maskNamesLocal.at(iii);
			}
		}

		// And load depth images
		{
			std::vector<std::string> depthNamesLocal;
		
			string inputDirectory = image_directory + "/depth";

			if (!getContents(inputDirectory, depthNamesLocal)) {
				std::printf("%s << No masks found! Skipping, and allowing system to synthesise them later..\n", __FUNCTION__);
			} else {

				//padNames(maskNamesLocal, true);
				sort(depthNamesLocal.begin(), depthNamesLocal.end());
				//unpadNames(maskNamesLocal);

				if (depthNamesLocal.size() != images->at(0).size()) {
					std::printf("%s << ERROR! Number of existing local images doesn't match number of available depth names.\n", __FUNCTION__);
					return false;
				}

				for (int iii = 0; iii < depthNamesLocal.size(); iii++) images->at(0).at(iii).depthName = depthNamesLocal.at(iii);
			}
		}
	
		std::printf("%s << (%d) images found\n", __FUNCTION__, int(images->at(0).size()));

	}

	return true;

}

void objectIdentifier::outputResults() {

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) { if (identityConfidenceData[aaa].is_open()) identityConfidenceData[aaa].close(); }

	cv::Mat resultsForFile, resultsForFile3;
	convertProbabilityMat(idPriorProb[MATCHING_METHODS_COUNT], resultsForFile);
	convertProbabilityMat(*idPriorProb3[MATCHING_METHODS_COUNT], resultsForFile3);

	/*
	cv::FileStorage fs(resultsOutputDir + "/identification_pp_matrices.yml", cv::FileStorage::WRITE);
	fs << "priorProbabilities" << resultsForFile;
	fs << "priorProbabilities3" << resultsForFile3;
	fs.release();
	*/

	if (resultsForFile.rows > 0) {
		string csv_file = resultsOutputDir + "/identification_pp_matrix.csv";
		writeMatrixToCSV(resultsForFile, csv_file);
		csv_file = resultsOutputDir + "/identification_pp_matrix_labeled.csv";
		writeMatrixToCSV(resultsForFile, csv_file, lib.objectFolders);
	}
	
	if (resultsForFile3.dims == 3) {
		string csv_file = resultsOutputDir + "/identification_pp_matrix_3.csv";
		writeMatrixToCSV(resultsForFile3, csv_file);
		csv_file = resultsOutputDir + "/identification_pp_matrix_3_labeled.csv";
		writeMatrixToCSV(resultsForFile3, csv_file, lib.objectFolders);
	}
}

void objectIdentifier::displayResults() {

	// Identification Results
	if ((cv::countNonZero(idSuccess[MATCHING_METHODS_COUNT]) + cv::countNonZero(idFailure[MATCHING_METHODS_COUNT])) != 0) {

		const int elements = 8;
		static double confidence_limits[elements] = { 1.00, 0.95, 0.90, 0.85, 0.80, 0.75, 0.50, 0.33 };

		int confidence_tallies[elements][2], confidence_tallies3[elements][2];
		for (int iii = 0; iii < elements; iii++) { 
			for (int jjj = 0; jjj < 2; jjj++) { 
				confidence_tallies[iii][jjj] = 0; 
				confidence_tallies3[iii][jjj] = 0; 
			} 
		}
	
		for (int iii = 0; iii < confidenceRecord[0].size(); iii++) {
			int idx = 0;
			while (confidenceRecord[1].at(iii) < confidence_limits[idx]) idx++;
			idx = min(idx, elements-1);
			confidence_tallies[idx][confidenceRecord[0].at(iii) == 1.0 ? 0 : 1]++;
		}

		for (int iii = 0; iii < confidenceRecord3[0].size(); iii++) {
			int idx = 0;
			while (confidenceRecord3[1].at(iii) < confidence_limits[idx]) idx++;
			idx = min(idx, elements-1);
			confidence_tallies3[idx][confidenceRecord3[0].at(iii) == 1.0 ? 0 : 1]++;
		}
	
		double confidence_performance[elements], confidence_performance3[elements];
		for (int iii = 0; iii < elements; iii++) {
			confidence_performance[iii] = double(confidence_tallies[iii][0]) / double(confidence_tallies[iii][1] + confidence_tallies[iii][0]);
			confidence_performance3[iii] = double(confidence_tallies3[iii][0]) / double(confidence_tallies3[iii][1] + confidence_tallies3[iii][0]);
		}

		int precision = 4;

		{
			std::cout << "Mean pairwise identification prior prob = (";

			double maxSum = 0.0, maxMean = 0.0;
			for (int iii = 0; iii < idPriorProb[0].rows; iii++) {
				for (int jjj = 0; jjj < idPriorProb[0].cols; jjj++) {
					if (idPriorProb[0].at<double>(iii,jjj) >= 0.0) maxSum += 1.0;
				}
			}
			maxMean = maxSum / double(idPriorProb[0].rows*idPriorProb[0].cols);

			for (int aaa = 0; aaa <= MATCHING_METHODS_COUNT; aaa++) {
				// Calculate mean identification result
				cv::Mat zeroedAbsences;
				convertProbabilityMat(idPriorProb[aaa], zeroedAbsences, true);

				cv::Scalar matSum = cv::sum( zeroedAbsences );
				double rawSum = matSum.val[0];
				double rawMean = rawSum/double(idPriorProb[aaa].rows*idPriorProb[aaa].cols);

				if (aaa < MATCHING_METHODS_COUNT) {
					std::cout << setw(precision+2) << std::setprecision(precision) << rawMean/maxMean;
					std::cout << ((aaa == (MATCHING_METHODS_COUNT-1)) ? ") " : ", ");
				} else {
					std::cout << endl;
					std::cout << "[" << setw(precision+2) << std::setprecision(precision) << rawMean/maxMean << "]";
				}
				
			}
			std::cout << " with " << int(maxMean*100.0) << "% testing coverage." << endl;

			double probabilityAccuracy = calculateProbabilityAccuracy(elements, confidence_limits, confidence_tallies);
			cout << "2-Way Test Probabilities Model Accuracy = " << setw(precision+2) << std::setprecision(precision) << probabilityAccuracy << endl;
		}

		std::cout << "Pair test success rate by confidence = " << endl;
		for (int iii = 0; iii < elements; iii++) {
			std::cout << "[>"; 
			std::cout << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << confidence_limits[iii] << "] ";
			if ((confidence_tallies[iii][1] + confidence_tallies[iii][0]) != 0) {
				std::cout << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') <<  confidence_performance[iii] << " " ;
			} else std::cout << "---- ";
			std::cout << "(" << (confidence_tallies[iii][0]+confidence_tallies[iii][1]) << ")" << endl;
		}
		std::cout << endl;

		{
			// Calculate mean identification result for 3-way tests
			cv::Mat zeroedAbsences;
			convertProbabilityMat(*idPriorProb3[MATCHING_METHODS_COUNT], zeroedAbsences, true);

			cv::Scalar matSum = cv::sum( zeroedAbsences );
			double rawSum = matSum.val[0];

			if (rawSum > 0.0) {
				double rawMean = rawSum/double(idPriorProb3[MATCHING_METHODS_COUNT]->size[0]*idPriorProb3[MATCHING_METHODS_COUNT]->size[1]*idPriorProb3[MATCHING_METHODS_COUNT]->size[2]);
	
				double maxMean = 0.0;
				for (int iii = 0; iii < idPriorProb3[MATCHING_METHODS_COUNT]->size[0]; iii++) {
					for (int jjj = 0; jjj < idPriorProb3[MATCHING_METHODS_COUNT]->size[1]; jjj++) {
						for (int kkk = 0; kkk < idPriorProb3[MATCHING_METHODS_COUNT]->size[2]; kkk++) {
							if (idPriorProb3[MATCHING_METHODS_COUNT]->at<double>(iii,jjj,kkk) >= 0.0) maxMean += 1.0;
						}
					}
				}
				maxMean /= double(idPriorProb3[MATCHING_METHODS_COUNT]->size[0]*idPriorProb3[MATCHING_METHODS_COUNT]->size[1]*idPriorProb3[MATCHING_METHODS_COUNT]->size[2]);
				std::cout << "Mean triplet identification prior prob = " << setw(precision+2) << std::setprecision(precision) << rawMean/maxMean << " with " << int(maxMean*100.0) << "% testing coverage." << endl;
				double probabilityAccuracy = calculateProbabilityAccuracy(elements, confidence_limits, confidence_tallies3);
				cout << "3-Way Test Probabilities Model Accuracy = " << setw(precision+2) << std::setprecision(precision) << probabilityAccuracy << endl;

				std::cout << "Triplet test success rate by confidence = " << endl;
				for (int iii = 0; iii < elements; iii++) {
					std::cout << "[>"; 
					std::cout << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << confidence_limits[iii] << "] ";
					if ((confidence_tallies3[iii][1] + confidence_tallies3[iii][0]) != 0) {
						std::cout << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') <<  confidence_performance3[iii] << " " ;
					} else std::cout << "---- ";
					std::cout << "(" << (confidence_tallies3[iii][0]+confidence_tallies3[iii][1]) << ")" << endl;
				}
				std::cout << endl;
			}
		}
	}

	// Registration Results
	{

	}
}

void imset::assign(const cv::Mat& img, const cv::Mat& mask, const cv::Mat& depth) {
	if (mask.type() == CV_8UC1) {
		mask.copyTo(inputMask);
	} else if (mask.type() == CV_8UC3) {
		cvtColor(mask, inputMask, cv::COLOR_BGR2GRAY);
	} else {
		std::printf("%s << ERROR! mask format unrecognized.\n", __FUNCTION__);
	}
	//histogramNormalize(img, inputImage, inputMask, 1, 1);
	img.copyTo(inputImage);
	//objectCount = countObjects(inputMask);
	blobs.resize(countBlobsInMask(inputMask));

	// Check if depth has been multiplied by 5...
	int multFactor = 5;
	for (int iii = 0; iii < depth.rows; iii++) {
		for (int jjj = 0; jjj < depth.cols; jjj++) {
			if (depth.at<unsigned short>(iii,jjj) % 5 != 0) {
				multFactor = 1;
				break;
			}
		}
		if (multFactor == 1) break;
	}

	inputDepth = depth / multFactor;
}

void imset::showFeatureImages(int idx, string custom_name) { 
	if (idx == -1) {
		for (int iii = 0; iii < blobs.size(); iii++) {
			char name[256];
			sprintf(name, "features[%d]", iii);
			cv::imshow(string(name), blobs[iii].featuresIm); 
		}
	} else {
		if (custom_name == "") custom_name = "features";
		cv::imshow(string(custom_name), blobs[idx].featuresIm); 

	}
}

void imset::describe(bool debug, string dir) {

	generateColourHistograms();
	extractCurrentContours();
	detectFeatures();
	classifyFeatures();
	describeFeatures();
	measureDimensions();

	if (debug) {
		generateFeatureImages(true);
		showFeatureImages();
		cv::waitKey(1);
	}
}

cv::Mat imset::getDebugIm(int blob_index, cv::Scalar color) {

	// Add features

	cv::Mat retImage;
	if (blobs.size() == 0) return retImage;

	(blob_index == -1) ? retImage = cv::Mat::zeros(DEBUG_SEGMENT_SIZE.height, DEBUG_SEGMENT_SIZE.width*int(blobs.size()), CV_8UC3) : retImage = cv::Mat::zeros(DEBUG_SEGMENT_SIZE, CV_8UC3);

	int alreadyAdded = 0;
	for (int iii = 0; iii < blobs.size(); iii++) {
		if ((blob_index == -1) || (blob_index == iii)) {
			cv::Mat mask = generateSpecificMask(inputMask, iii);
			
			if (mask.rows == 0) continue;
			
			cv::Mat maskedMat;
			inputImage.copyTo(maskedMat, mask);
			
			// If features available, draw them here
			cv::Mat featureIm;
			if ((blobs[iii].foregroundFeatures.size() > 0) || (blobs[iii].edgeFeatures.size() > 0)) {
				displayKeyPoints(maskedMat, blobs[iii].foregroundFeatures, blobs[iii].edgeFeatures, featureIm);
			} else maskedMat.copyTo(featureIm);

			// If requested, draw contour
			if (color(0) != -1.0) {
				vector<vector<cv::Point> > contour_vec;
				contour_vec.push_back(blobs[iii].contour);
				cv::drawContours(featureIm, contour_vec, 0, color, 4);
			}
			
			cv::Mat croppedMat = cropImage(featureIm);
			cv::Mat buffIm = buffImage(croppedMat);
			cv::Mat dst_roi = retImage(cv::Rect(alreadyAdded, 0, DEBUG_SEGMENT_SIZE.width, DEBUG_SEGMENT_SIZE.height));
			buffIm.copyTo(dst_roi);
			alreadyAdded += DEBUG_SEGMENT_SIZE.width;
		}
		
	}
	return retImage;
}

void objectIdentifier::clearPreviousResults() {
	currentBlob = -1;
    currentViewLabels.clear();
	currentVersionNames.clear();
    currentViewRotations.clear();
	currentConfidence = -1.0;
	currentPreConfidence = -1.0;
}

int imageDirectory::findObjectIndex(string object_name) {
	for (int iii = 0; iii < objectFolders.size(); iii++) {
		if (objectFolders.at(iii) == object_name) return iii;
	}
	return -1;
}

int countDifferentLevels(vector<imset> *image_sets) {
	vector<int> unique_indices;

	for (int iii = 0; iii < image_sets->size(); iii++) {
		int idx1 = -1;
		int idx2 = -1;
		getNPVal(image_sets->at(iii).getDescName(0), idx1, idx2);
		bool isUnique = true;
		for (int jjj = 0; jjj < unique_indices.size(); jjj++) {
			if (unique_indices.at(jjj) == idx1) {
				isUnique = false;
				break;
			}
		}
		if (isUnique) unique_indices.push_back(idx1);
	}

	return int(unique_indices.size());
}

int imageDirectory::findViewIndex(string object_name, string image_name) {

	size_t index = 0;
	index = image_name.find(".", index);
	image_name.replace(index, 4, ".yml");

	int object_index = -1;
	for (int iii = 0; iii < objectFolders.size(); iii++) {
		if (objectFolders.at(iii) == object_name) {
			object_index = iii;
			for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {
				string vN = images->at(iii).at(jjj).blobs[0].viewName;
				if (((vN + ".jpg") == image_name) || ((vN + ".png") == image_name)) return jjj;
			}
			break;
		}
	}

	int test_level_index = -1, test_frame_index = -1;
	getNPVal(image_name, test_level_index, test_frame_index);
	
	int level_count = countDifferentLevels(&images->at(object_index));
	int frame_count = int(images->at(object_index).size()) / level_count;
	
	double minError = std::numeric_limits<double>::max();
	int minIndex = -1;

	for (int iii = 0; iii < images->at(object_index).size(); iii++) {
		// Find the closest matching index if none actually exist in the library
		int level_index = -1, frame_index = -1;
		getNPVal(images->at(object_index).at(iii).blobs[0].viewName + ".yml", level_index, frame_index);

		double levelDiff = calculateViewError(test_level_index, test_frame_index, level_index, frame_index, level_count, frame_count);

		if (levelDiff < minError) {
			minError = levelDiff;
			minIndex = iii;
		}
	}

	return minIndex;
}

void objectIdentifier::generateFailureImage(const vector<string>& objects, bool is_correct) {
	
	currentTest.matchedIndices.clear();
	currentTest.matchedIndices.resize(testImage.blobs.size());

	for (int blob_idx = 0; blob_idx < testImage.blobs.size(); blob_idx++) {

		for (int class_idx = 0; class_idx < testImage.blobs.size(); class_idx++) {

			vector<double> local_scores[MATCHING_METHODS_COUNT];
			vector<double> confidence[MATCHING_METHODS_COUNT]; // stores the ranking confidence for each method

			int class_lib_idx = lib.findObjectIndex(objects.at(class_idx));

			for (int mm_idx = 0; mm_idx < MATCHING_METHODS_COUNT; mm_idx++) {
		
				// Copy vector of scores paired with indices
				double mean = 0.0;
				int valid_count = 0;
				for (int iii = 0; iii < blobMatchScores[mm_idx].at(blob_idx).at(class_lib_idx).size(); iii++) {
					double val(blobMatchScores[mm_idx].at(blob_idx).at(class_lib_idx).at(iii));
					if (val != std::numeric_limits<double>::max()) {
						mean += val;
						valid_count++;
					}
					local_scores[mm_idx].push_back(val);
				}
				if (valid_count > 0) mean /= double(valid_count);

				// Estimate the pseudo-confidence of the rankings for each method by how much superior it is to its mean
				for (int iii = 0; iii < local_scores[mm_idx].size(); iii++) {
					confidence[mm_idx].push_back(mean/local_scores[mm_idx].at(iii));
				}

			}

			// Combine the confidences to get one vector
			vector<double> pseudoConfidences;
			for (int iii = 0; iii < confidence[0].size(); iii++) pseudoConfidences.push_back(0.0);

			for (int mm_idx = 0; mm_idx < MATCHING_METHODS_COUNT; mm_idx++) {
				for (int iii = 0; iii < confidence[mm_idx].size(); iii++) {
					pseudoConfidences.at(iii) += confidence[mm_idx].at(iii);
				}
			}

			// Sort this vector to get the final indices
			vector<score_pair> combined_scores;
			for (int iii = 0; iii < confidence[0].size(); iii++) {
				score_pair sp(pseudoConfidences.at(iii), iii);
				combined_scores.push_back(sp);
			}
			std::sort(combined_scores.begin(), combined_scores.end());

			// Larger value is better with these pseudo-confidences!
			currentTest.matchedIndices.at(blob_idx).push_back(combined_scores.at(combined_scores.size()-1).second);

		}
	}

	cv::Mat debugImage;

	testImage.generateFeatureImages();
	
	// Then add best class matches for each blob
	for (int blob_idx = 0; blob_idx < testImage.blobs.size(); blob_idx++) {

		addToDebugImage(testImage.blobs[blob_idx].croppedBlob, debugImage, 0, blob_idx);

		cv::Mat testIm = testImage.getDebugIm(blob_idx);
		addToDebugImage(testIm, debugImage, 1, blob_idx);

		for (int class_idx = 0; class_idx < testImage.blobs.size(); class_idx++) {
			int class_lib_idx = lib.findObjectIndex(objects.at(class_idx));
			
			int best_match_idx = currentTest.matchedIndices.at(blob_idx).at(class_idx);

			if ((best_match_idx < 0) || (best_match_idx >= lib.images->at(class_lib_idx).size())) {
				cout << "[" << __FUNCTION__ << "] ERROR! Wasn't able to obtain a valid index for the best matching view." << endl;
			}

			lib.loadImage(class_lib_idx,best_match_idx);

			cv::Scalar color;
			if ((is_correct && (blob_idx == class_idx)) || (!is_correct & (blob_idx != class_idx))) {
				color = cv::Scalar(0, 255, 0);
			} else color = cv::Scalar(0, 0, 255);
			cv::Mat classImage = lib.images->at(class_lib_idx).at(best_match_idx).getDebugIm(0, color);

			addToDebugImage(classImage, debugImage, class_idx+2, blob_idx);
		}
	}

	// Then display image and save to directory
	imshow("debugImage", debugImage);
	cv::waitKey(1);

	stringstream ss;
	ss << resultsOutputDir << "/error_images/im_" << setfill('0') << setw(6) << counter << ".jpg";
	cv::imwrite(ss.str(), debugImage);
}

void objectIdentifier::findMatchingViews(int blobIndex, int bestLibraryIndex, vector<string>& bestViewLabels, vector<vector<double> >& bestRotations, double& preconf, int num_views, bool debug, bool avoid_best) {

	if (debug) cout << "[" << __FUNCTION__ << "] Entered." << endl;

    bestViewLabels.clear();

	vector<double> local_scores[MATCHING_METHODS_COUNT];
	vector<double> confidence[MATCHING_METHODS_COUNT]; // stores the ranking confidence for each method

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		
		// Copy vector of scores paired with indices
		double mean = 0.0;
		int valid_count = 0;
		
		if (debug) cout << "[" << __FUNCTION__ << "] blobMatchScores[" << aaa << "].at(" << blobIndex << ").at(" << bestLibraryIndex << ").size() = " << blobMatchScores[aaa].at(blobIndex).at(bestLibraryIndex).size() << endl;
		
		for (int iii = 0; iii < blobMatchScores[aaa].at(blobIndex).at(bestLibraryIndex).size(); iii++) {
			double val(blobMatchScores[aaa].at(blobIndex).at(bestLibraryIndex).at(iii));
			if (val != std::numeric_limits<double>::max()) {
				mean += val;
				valid_count++;
			}
			local_scores[aaa].push_back(val);
		}
		if (valid_count > 0) mean /= double(valid_count);

		// Estimate the confidences of the rankings
		vector<score_pair> distances;
		for (int iii = 0; iii < local_scores[aaa].size(); iii++) {
			score_pair sp(local_scores[aaa].at(iii), iii);
			distances.push_back(sp);
		}
		std::sort(distances.begin(), distances.end());

		confidence[aaa].resize(distances.size());

		if (distances.size() < 2) {
			confidence[aaa].at(0) = 1.0;
		} else {
			for (int iii = 0; iii < distances.size(); iii++) {
				// double conf = mean/local_scores[aaa].at(iii); // Estimate the pseudo-confidence of the rankings for each method by how much superior it is to its mean
				double ratio = (iii == 0) ? local_scores[aaa].at(distances.at(iii).second)/local_scores[aaa].at(distances.at(1).second) : local_scores[aaa].at(distances.at(iii).second)/local_scores[aaa].at(distances.at(0).second);
				double conf = calculateConfidence(ratio, viewRatioPoints[aaa]);
				confidence[aaa].at(distances.at(iii).second) = conf;
			}
		}
	}

	// Combine the confidences to get one vector
	vector<double> pseudoConfidences;
	for (int iii = 0; iii < confidence[0].size(); iii++) pseudoConfidences.push_back(0.0);

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		if (!is_contained(aaa, amm)) continue;
		for (int iii = 0; iii < confidence[aaa].size(); iii++) {
			pseudoConfidences.at(iii) += confidence[aaa].at(iii);
		}
	}
	
	if (debug) cout << "[" << __FUNCTION__ << "] confidence[0].size() = " << confidence[0].size() << endl;

	// Sort this vector to get the final indices
	vector<score_pair> combined_scores;
	for (int iii = 0; iii < confidence[0].size(); iii++) {
		score_pair sp(pseudoConfidences.at(iii), iii);
		combined_scores.push_back(sp);
	}
	std::sort(combined_scores.begin(), combined_scores.end());

	if (debug) cout << "[" << __FUNCTION__ << "] combined_scores.size() = " << combined_scores.size() << endl;
	// Store the best N indices - Larger value is better with these pseudo-confidences!
	if (combined_scores.size() > num_views) combined_scores.erase(combined_scores.begin(), combined_scores.end()-num_views);
	if (debug) cout << "[" << __FUNCTION__ << "] combined_scores trimmed to = " << combined_scores.size() << endl;
	
    bestRotations.resize(combined_scores.size());
    
    for (int iii = 0; iii < combined_scores.size(); iii++) {
        
        if (debug) cout << "[" << __FUNCTION__ << "] A (" << iii << ")" << endl;
        
        if (iii >= lib.images->at(bestLibraryIndex).size()) {
			bestViewLabels.push_back("NULL");
			for (int jjj = 0; jjj < 9; jjj++) bestRotations.at(iii).push_back( ((jjj == 0) || (jjj == 4) || (jjj == 8)) ? 1.0 : 0.0);
			continue;
		}
		
		if (debug) cout << "[" << __FUNCTION__ << "] B" << endl;
        
        // combined_scores.at(combined_scores.size()-iii-1).second EXCEEDS LENGTH!
        
        if (debug) {
			cout << "[" << __FUNCTION__ << "] bestLibraryIndex = " << bestLibraryIndex << endl;
			cout << "[" << __FUNCTION__ << "] combined_scores.size()-iii-1 = " << combined_scores.size()-iii-1 << endl;
			cout << "[" << __FUNCTION__ << "] combined_scores.at(combined_scores.size()-iii-1).second = " << combined_scores.at(combined_scores.size()-iii-1).second << endl;
			cout << "[" << __FUNCTION__ << "] lib.images->size() = " << lib.images->size() << endl;
			cout << "[" << __FUNCTION__ << "] lib.images->at(bestLibraryIndex).size() = " << lib.images->at(bestLibraryIndex).size() << endl;
		}
		
        bestViewLabels.push_back(lib.images->at(bestLibraryIndex).at(combined_scores.at(combined_scores.size()-iii-1).second).blobs[0].viewName);

		if (debug) cout << "[" << __FUNCTION__ << "] C" << endl;

        size_t rotSize = lib.images->at(bestLibraryIndex).at(iii).blobs[0].rotation.size();
        if (rotSize == 0) {
            for (int jjj = 0; jjj < 9; jjj++) {
                bestRotations.at(iii).push_back( ((jjj == 0) || (jjj == 4) || (jjj == 8)) ? 1.0 : 0.0);
            }
        } else if (rotSize < 9) {
            cout << "ERROR! There seems to be an invalid number of elements in the rotation field of the matched view." << endl;
        } else {
            for (int jjj = 0; jjj < 9; jjj++) {
                bestRotations.at(iii).push_back(lib.images->at(bestLibraryIndex).at(iii).blobs[0].rotation.at(jjj));
            }
        }
    }
	
	if (debug) cout << "[" << __FUNCTION__ << "] About to generate preconf..." << endl;
	if (lib.images->at(bestLibraryIndex).at(0).blobs[0].rotation.size() == 0) {
		preconf = 0.0;
	} else if (combined_scores.size() == 1) {
		preconf = 1.0;
	} else preconf = max(0.0, min(1.0, (combined_scores.at(combined_scores.size()-2).first-combined_scores.at(combined_scores.size()-1).first)/combined_scores.at(combined_scores.size()-2).first));

}

cv::Mat objectIdentifier::generateDisplayImage(cv::Rect roiRect) {
	cv::Mat retMat;

	retMat = testImage.inputImage;

	for (int iii = 0; iii < testImage.blobs.size(); iii++) {
		vector<vector<cv::Point> > contour_vec;
		contour_vec.push_back(testImage.blobs[iii].contour);
		cv::Scalar color = (iii == currentBlob) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
		cv::drawContours(retMat, contour_vec, 0, color, 4);
	}
	
	if (roiRect.area() > 0) rectangle(retMat, roiRect.tl(), roiRect.br(), cv::Scalar(255, 100, 100), 2);
	
	return retMat;
}

bool objectIdentifier::outputConfidenceData(const vector<std::string>& objectNames, vector<vector<vector<double> > > *blobMatchScoreVector) {

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) { 
		if (!identityConfidenceData[aaa].is_open()) {
			stringstream ss;
			ss << resultsOutputDir << "/identity_confidence_data_" << aaa << ".txt";
			confidence_file[aaa] = ss.str();

			identityConfidenceData[aaa].open(confidence_file[aaa].c_str()); 
		}
	}

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {

		for (int iii = 0; iii < objectNames.size(); iii++) { // For each blob in test im

			int true_index = lib.findObjectIndex(objectNames.at(iii));

			if (true_index == -1) {
				std::printf("%s << ERROR! Unable to find object (%s). Please check if you have a the correct YML library loaded.\n", __FUNCTION__, objectNames.at(iii).c_str());
				return false;
			}

			vector<vector<vector<double> > > x = blobMatchScoreVector[aaa];

			double val = *(std::min_element(blobMatchScoreVector[aaa].at(iii).at(true_index).begin(), blobMatchScoreVector[aaa].at(iii).at(true_index).end()));

			for (int jjj = 0; jjj < blobMatchScoreVector[aaa].at(iii).size(); jjj++) { // For each library blob attempted to match

				if (true_index == jjj) continue;
				
				//printf("length: %d\n", blobMatchScoreVector[aaa].at(iii).at(jjj).size());
				if(blobMatchScoreVector[aaa].at(iii).at(jjj).size() == 0){
					printf("Warning. Item %s appears to have no yml data.\n", lib.objectFolders.at(jjj).c_str());
					continue;
				}
				double secondary = *(std::min_element(blobMatchScoreVector[aaa].at(iii).at(jjj).begin(), blobMatchScoreVector[aaa].at(iii).at(jjj).end()));

				// Calculate ratios and only output true ones (false ones should be inverses, right?)

				if (secondary == 0) printf("%s << ERROR!\n", __FUNCTION__);

				double ratio = val/secondary;

				// if (ratio == 0.0) printf("%s << WARNING! NNDR calculated as zero!!\n", __FUNCTION__);

				if ((val != 0) && (val != std::numeric_limits<double>::max()) && (secondary != 0) && (secondary != std::numeric_limits<double>::max())) {
					identityConfidenceData[aaa] << ratio << endl;
				}
			}
		}

	}

	return true;
}

size_t imageDirectory::getTotalImageCount() { 
	size_t count = 0;
	for (int iii = 0; iii < images->size(); iii++) count += images->at(iii).size();
	return count; 
} 

size_t imageDirectory::getTotalBlobCount() { 

	if (directoryFormat == SINGLE_OBJECT_FORMAT) return getTotalImageCount();

	size_t count = 0;
	for (int iii = 0; iii < images->size(); iii++) {
		for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {
			loadImage(iii, jjj);
			cv::Mat rgbImage, maskImage, depthImage;
			retrieveImage(iii, jjj, rgbImage, maskImage, depthImage);
			count += countBlobsInMask(maskImage);

		}
	}
	return count; 
} 


void objectLibrary::clearPreviousResults() {
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) blobMatchScores[aaa].clear();
}

bool objectLibrary::fullyMatch(const blob& input_blob, vector<string> potential_objects, bool debug) {

	vector<string> objectsToCompare;

	bool deepDebug = false;

	if (potential_objects.size() == 0) {
		objectsToCompare = objectFolders; 
	} else {
		vector<string> unique_objects;
		for (int iii = 0; iii < potential_objects.size(); iii++) {
			bool isUnique = true;
			for (int jjj = 0; jjj < unique_objects.size(); jjj++) {
				if (potential_objects.at(iii) == unique_objects.at(jjj)) {
					isUnique = false;
					break;
				}
			}
			if (isUnique) unique_objects.push_back(potential_objects.at(iii));
		}
		
		objectsToCompare = unique_objects;
	}

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {

		blobMatchScores[aaa].clear();
		blobMatchScores[aaa].resize(objectFolders.size());

		for (int iii = 0; iii < objectsToCompare.size(); iii++) {

			int object_index = findObjectIndex(objectsToCompare.at(iii));
			
			if (debug) cout << "[" << __FUNCTION__ << "] [aaa = " << aaa << "] objectsToCompare.at(" << iii << ") = " << objectsToCompare.at(iii) << "; object_index = " << object_index << endl;
			
			if (object_index < 0) cout << "[" << __FUNCTION__ << "] ERROR! " << "Could not find index for object <" << objectsToCompare.at(iii) << ">" << endl;

			// if (debug && (object_index == 7)) cout << "[" << __FUNCTION__ << "] [aaa = " << aaa << "] objectsToCompare.at(" << iii << ") = " << objectsToCompare.at(iii) << "; object_index = " << object_index << endl;

			for (int jjj = 0; jjj < images->at(object_index).size(); jjj++) {
				blob library_blob = images->at(object_index).at(jjj).blobs[0];
				double matchingScore = calculateMatchingScore(input_blob, library_blob, aaa);
				if (deepDebug && (images->at(object_index).at(jjj).getDebugIm(0).rows != 0)) {
					imshow("libraryBlob", images->at(object_index).at(jjj).getDebugIm(0));
					cv::waitKey(0);
				}

 				blobMatchScores[aaa].at(object_index).push_back(matchingScore);
			}
		}
	}
	return true;
}

void objectIdentifier::supplyPrematchingResults(vector<vector<vector<double> > > *blobMatchScoreVector) {
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		blobMatchScores[aaa].clear();
		blobMatchScores[aaa].resize(blobMatchScoreVector[aaa].size());
		for (int iii = 0; iii < blobMatchScoreVector[aaa].size(); iii++) {
			blobMatchScores[aaa].at(iii).resize(blobMatchScoreVector[aaa].at(iii).size());
			for (int jjj = 0; jjj < blobMatchScoreVector[aaa].at(iii).size(); jjj++) {
				for (int kkk = 0; kkk < blobMatchScoreVector[aaa].at(iii).at(jjj).size(); kkk++) {
					blobMatchScores[aaa].at(iii).at(jjj).push_back(blobMatchScoreVector[aaa].at(iii).at(jjj).at(kkk));
				}
			}
		}
	}
}

bool objectIdentifier::processMatchingResults(const vector<std::string>& objectNames, const vector<std::string>& viewNames, vector<vector<vector<double> > > *blobMatchScoreVector) {

	double *minScore[MATCHING_METHODS_COUNT];
	int *best_index[MATCHING_METHODS_COUNT], *best_jjj[MATCHING_METHODS_COUNT];

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		minScore[aaa] = new double[objectNames.size()];
		best_index[aaa] = new int[objectNames.size()];
		best_jjj[aaa] = new int[objectNames.size()];
	}

	for (int iii = 0; iii < objectNames.size(); iii++) {	
		for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
			minScore[aaa][iii] = std::numeric_limits<double>::max();
			best_index[aaa][iii] = -1;
			best_jjj[aaa][iii] = -1;
		}
	}

	vector<vector<int> > bestBlobMatchIndices[MATCHING_METHODS_COUNT];

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {

		bestBlobMatchIndices[aaa].resize(objectNames.size());

		if (identificationMatchingScoreMatrix[aaa].rows == 0) identificationMatchingScoreMatrix[aaa] = cv::Mat::zeros(int(lib.objectFolders.size()), int(lib.objectFolders.size()), CV_64FC1);
		if (testCountMatrix[aaa].rows == 0) testCountMatrix[aaa] = cv::Mat::zeros(int(lib.objectFolders.size()), int(lib.objectFolders.size()), CV_64FC1);
		
		for (int iii = 0; iii < objectNames.size(); iii++) {	
			int test_index = lib.findObjectIndex(objectNames.at(iii));

			if (test_index == -1) {
				printf("%s << ERROR! Was unable to find object <%s> in the YML library. Are you using the correct library?\n", __FUNCTION__, objectNames.at(iii).c_str());
				return false;	
			}

			for (int jjj = 0; jjj < blobMatchScoreVector[aaa].at(iii).size(); jjj++) {

				vector<double> x = blobMatchScoreVector[aaa].at(iii).at(jjj);

				//printf("length: %d\n", blobMatchScoreVector[aaa].at(iii).at(jjj).size());
				if(blobMatchScoreVector[aaa].at(iii).at(jjj).size() == 0){
					printf("Warning. Item %s appears to have no yml data.\n", lib.objectFolders.at(jjj).c_str());
					continue;
				}
				double bestScore = *(std::min_element(blobMatchScoreVector[aaa].at(iii).at(jjj).begin(), blobMatchScoreVector[aaa].at(iii).at(jjj).end()));

				int bestIndex = int(distance(blobMatchScoreVector[aaa].at(iii).at(jjj).begin(),min_element(blobMatchScoreVector[aaa].at(iii).at(jjj).begin(),blobMatchScoreVector[aaa].at(iii).at(jjj).end())));

				//if (bestIndex > 2) {
				//	int a = 1;
				//}

				bestBlobMatchIndices[aaa].at(iii).push_back(bestIndex);

				identificationMatchingScoreMatrix[aaa].at<double>(test_index, jjj) += bestScore;
				testCountMatrix[aaa].at<double>(test_index, jjj) += 1.0;

				if ((bestScore > 0.0) && (bestScore <= minScore[aaa][iii])) {
					minScore[aaa][iii] = bestScore;
					//printf("[%04d][%04d][%04d] = (%5.5f)\n", iii, jjj, aaa, blobMatchScoreVector[aaa].at(iii).at(jjj));
					best_index[aaa][iii] = test_index;
					best_jjj[aaa][iii] = jjj;
				}

			}
		}
	}

	for (int iii = 0; iii < objectNames.size(); iii++) {	
		//printf("   < %*.*s / %*.*s >\n", 15, 15, objectNames.at(iii).c_str(), 7, 7, viewNames.at(iii).c_str());
		for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
			int best_jai = best_jjj[aaa][iii];

			if (best_jai == -1) {
				printf("%s << ERROR! None of the library objects got a valid best match score for this blob.\n", __FUNCTION__);
				break;
			}

			int bmiv_jai = bestBlobMatchIndices[aaa].at(iii).at(best_jai);

			if (bmiv_jai == -1) {
				printf("%s << ERROR! There doesn't seem to be any valid corresponding view match for this blob.\n", __FUNCTION__);
				break;
			}

			//printf("%d) < %*.*s / %*.*s >\n", aaa, 15, 15, lib.objectFolders.at(best_jai).c_str(), 7, 7, lib.images->at(best_jai).at(bmiv_jai).blobs[0].viewName.c_str());
		}
		//printf("\n");
	}

	return true;
}

void objectLibrary::retrieveMatchingResults(vector<vector<vector<double> > > *blobMatchScoreVector) { 

	int current_index = int(blobMatchScoreVector[0].size());

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		blobMatchScoreVector[aaa].resize(current_index+1);
		blobMatchScoreVector[aaa].at(current_index).resize(blobMatchScores[aaa].size());
		for (int iii = 0; iii < blobMatchScores[aaa].size(); iii++) {
			for (int jjj = 0; jjj < blobMatchScores[aaa].at(iii).size(); jjj++) {
				blobMatchScoreVector[aaa].at(current_index).at(iii).push_back(blobMatchScores[aaa].at(iii).at(jjj));
			}
		}
	}
}

void objectLibrary::retrieveSpecificMatchingResults(vector<double> *blobMatchScoreVector, int object_idx) { 
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		for (int iii = 0; iii < blobMatchScores[aaa].at(object_idx).size(); iii++) {
			blobMatchScoreVector[aaa].push_back(blobMatchScores[aaa].at(object_idx).at(iii));
		}
	}
}

void objectIdentifier::saveCurrentBlobMatchScores() {

	ofstream blobMatchScoresFile;
	string filename = "test.txt";
		
	blobMatchScoresFile.open(filename.c_str());

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		for (int iii = 0; iii < blobMatchScores[aaa].size(); iii++) {
			for (int jjj = 0; jjj < blobMatchScores[aaa].at(iii).size(); jjj++) { 
				for (int kkk = 0; kkk < blobMatchScores[aaa].at(iii).at(jjj).size(); kkk++) { 
					//cout << "bMS[" << aaa << "](" << iii << "," << jjj << "," << kkk << ") = " << blobMatchScores[aaa].at(iii).at(jjj).at(kkk) << endl;
					blobMatchScoresFile << "bMS[" << aaa << "](" << iii << "," << jjj << "," << kkk << ") = " << blobMatchScores[aaa].at(iii).at(jjj).at(kkk) << endl;
				}
			}
		}
	}

	blobMatchScoresFile.close();
}

void objectIdentifier::displayCurrentMatchingResults() {

	if (testImage.inputImage.rows > 0) imshow("currentBlobs", testImage.getDebugIm(-1));
	
	for (int iii = 0; iii < idPriorProb[MATCHING_METHODS_COUNT].rows; iii++) {
		if (idPriorProb[MATCHING_METHODS_COUNT].at<double>(iii,iii) == -1.0) {
			idPriorProb[MATCHING_METHODS_COUNT].at<double>(iii,iii) = -2.0;	
		} else if ((idPriorProb[MATCHING_METHODS_COUNT].at<double>(iii,iii) != 1.0) && (idPriorProb[MATCHING_METHODS_COUNT].at<double>(iii,iii) != -2.0)) {
			cout << __FUNCTION__ << " < ERROR! The system seems to have failed when both blobs were of the same class. (" << idPriorProb[MATCHING_METHODS_COUNT].at<double>(iii,iii) << ")" << endl;
			cout << "idPriorProb = " << idPriorProb << endl;
			cout << "idSuccess = " << idSuccess << endl;
			cout << "idFailure = " << idFailure << endl;
		}
	}

	expandAndDisplay(idPriorProb[MATCHING_METHODS_COUNT], "idPriorProb", 1.0);

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		
		cv::Mat averageMatchingScore = identificationMatchingScoreMatrix[aaa] / testCountMatrix[aaa];
		for (int iii = 0; iii < testCountMatrix[aaa].rows; iii++) {
			for (int jjj = 0; jjj < testCountMatrix[aaa].cols; jjj++) {
				if (testCountMatrix[aaa].at<double>(iii,jjj) == 0) averageMatchingScore.at<double>(iii,jjj) = -1.0;
				if (testCountMatrix[aaa].at<double>(iii,jjj) == std::numeric_limits<double>::max()) averageMatchingScore.at<double>(iii,jjj) = -2.0;
			}
		}

		/*
		stringstream windowName;
		windowName << "iMSM[" << aaa << "]";
		expandAndDisplay(averageMatchingScore, windowName.str());
		*/
	}

	cv::waitKey(1);
}

int objectIdentifier::findMatchingIdentity(const vector<std::string>& objectNames, double& confidence, int searchObjectIndex, bool debug, int *blob_indices, double *blob_confidences) {

	// debug = true;

	if (blob_indices == NULL) blob_indices = new int[MATCHING_METHODS_COUNT];
	if (blob_confidences == NULL) blob_confidences = new double[MATCHING_METHODS_COUNT];

	if (debug) cout << "[" << __FUNCTION__ << "] 0." << endl;

	vector<int> identicalSearchObjectIndices;
	for (int iii = 0; iii < objectNames.size(); iii++) {
		if (objectNames.at(iii) == objectNames.at(searchObjectIndex)) identicalSearchObjectIndices.push_back(iii);
	}
	
	if (debug) cout << "[" << __FUNCTION__ << "] 1." << endl;

	if (identityConfidencePoints[0].size() == 0) loadConfidencePoints();

	size_t blob_count = (testImage.blobs.size() == 0) ? objectNames.size() : testImage.blobs.size();
	size_t class_count = blob_count;
	
	if (debug) cout << "[" << __FUNCTION__ << "] A." << endl;

	// B: Distances for each blob to each class using each metric
	cv::Mat bestDistances[MATCHING_METHODS_COUNT];
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		bestDistances[aaa] = cv::Mat::zeros(int(blob_count), int(class_count), CV_64FC1);

		for (int kkk = 0; kkk < blob_count; kkk++) { // For each blob
			for (int iii = 0; iii < objectNames.size(); iii++) { // For each potential identity

				// Find index in library that matches this object
				int object_index = lib.findObjectIndex(objectNames.at(iii));

				if (object_index == -1) {
					std::printf("%s << ERROR! Unable to find object (%s)\n", __FUNCTION__, objectNames.at(iii).c_str()); 
					return -1;
				}
				bestDistances[aaa].at<double>(kkk,iii) = *(std::min_element(blobMatchScores[aaa].at(kkk).at(object_index).begin(), blobMatchScores[aaa].at(kkk).at(object_index).end()));
			}
		}
	}
	
	if (debug) cout << "[" << __FUNCTION__ << "] B." << endl;
	
	if (debug) { for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) cout << "bestDistances[" << aaa << "] = " << endl << bestDistances[aaa] << endl; }

	// C: Generate pairwise probability matrices for each blob for each metric
	vector<cv::Mat> pairwiseProbs[MATCHING_METHODS_COUNT];
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		for (int kkk = 0; kkk < blob_count; kkk++) { // For each blob
			cv::Mat pairwiseMat = cv::Mat::zeros(int(class_count), int(class_count), CV_64FC1);
			
			for (int mmm = 0; mmm < class_count; mmm++) { // For each primary class
				for (int nnn = 0; nnn < class_count; nnn++) { // For each secondary class

					double pairwiseProbability = 1.0;
					if (mmm != nnn) {
						// Get the two relevant scores and ratio
						double primaryScore = bestDistances[aaa].at<double>(kkk,mmm);
						double secondaryScore = bestDistances[aaa].at<double>(kkk,nnn);
						double ratio = primaryScore/secondaryScore;
						if (primaryScore == secondaryScore) {
							ratio = 1.0;
						} else if (primaryScore == std::numeric_limits<double>::max()) {
							ratio = 1.0; // theoretically should be std::numeric_limits<double>::max()
						} else if (secondaryScore == std::numeric_limits<double>::max()) {
							ratio = 1.0; // theoretically should be 0
						}
						
						pairwiseProbability = calculateConfidence(ratio, identityConfidencePoints[aaa]);
						//pairwiseProbability = 0.0;
					}
					
					pairwiseMat.at<double>(mmm,nnn) = pairwiseProbability;
				}
			}

			if (debug) cout << "pairwiseMat[" << aaa << "](" << kkk << ") = " << endl << pairwiseMat << endl; 
			pairwiseProbs[aaa].push_back(pairwiseMat);
		}
	}

	// D: Generate local prior probability matrices, by incorporating presence of multiple classes
	cv::Mat localPriorProbs[MATCHING_METHODS_COUNT];
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {

		localPriorProbs[aaa] = cv::Mat::zeros(int(blob_count), int(class_count), CV_64FC1);

		for (int kkk = 0; kkk < blob_count; kkk++) { // For each blob
			
			double denom = 0.0;
			
			vector<double> products;
			for (int mmm = 0; mmm < class_count; mmm++) { // For each primary class
				double prod = 1.0;
				for (int nnn = 0; nnn < class_count; nnn++) { // For each secondary class
					prod *= pairwiseProbs[aaa].at(kkk).at<double>(mmm,nnn);
				}
				denom += prod;
				products.push_back(prod);
			}

			for (int mmm = 0; mmm < class_count; mmm++) { // For each primary class
				localPriorProbs[aaa].at<double>(kkk,mmm) = denom != 0 ? products.at(mmm)/denom : 1.0/double(class_count);
			}

		}

		if (debug) cout << "localPriorProbs[" << aaa << "] = " << endl << localPriorProbs[aaa] << endl; 
	}

	// F: Generate different possible matrix configurations
	vector<cv::Mat> possibleConfigs;
	generateIdentityConfigurations(possibleConfigs, int(blob_count), int(class_count));

	// Iterate through each matching method in order of effectiveness, only if results warrant it 
	cv::Mat localPriorProbsCombined = cv::Mat::zeros(int(blob_count), int(class_count), CV_64FC1);
	int methodsConsidered = 0;
	double bestProbability = 0.0;
	int absoluteBestBlobIndex = -1;

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		blob_confidences[aaa] = 0.0;
		blob_indices[aaa] = -1;

		// E: Combine probabilities from different methods into 1 single matrix
		if (!is_contained(aaa, amm)) continue;

		// Clear to only consider one at a time
		{
			localPriorProbsCombined = cv::Mat::zeros(int(blob_count), int(class_count), CV_64FC1);
			methodsConsidered = 0;
		}

		localPriorProbsCombined *= double(methodsConsidered);
		localPriorProbsCombined += localPriorProbs[aaa];
		methodsConsidered++;
		localPriorProbsCombined /= double(methodsConsidered);

		if (debug) cout << "localPriorProbsCombined = " << endl << localPriorProbsCombined << endl; 

		// E2: Using old method, find the blob which has the highest local prob of being the class of interest
		if (debug) {
			int oldBestBlob = -1;
			double oldBestBlobProb = 0.0;
			for (int iii = 0; iii < blob_count; iii++) {
				double oldBestBlobProbLocal = 0.0;
				for (int kkk = 0; kkk < identicalSearchObjectIndices.size(); kkk++) {
					oldBestBlobProbLocal += localPriorProbsCombined.at<double>(iii, identicalSearchObjectIndices.at(kkk));
				}
				if (oldBestBlobProbLocal > oldBestBlobProb) {
					oldBestBlobProb = oldBestBlobProbLocal;
					oldBestBlob = iii;
				}
			}

			cout << "oldBestMethod: (" << oldBestBlob << ") with prob = " << oldBestBlobProb << endl;
		}

		// G: Determine probabilities of each configuration
		vector<double> configProbs;
		double freqSumm = 0.0;
		for (int iii = 0; iii < possibleConfigs.size(); iii++) {
			cv::Mat localMat = localPriorProbsCombined * possibleConfigs.at(iii);
			//cout << "localMat = (" << iii << ") = " << endl << localMat << endl; 
			double freq = 1.0;
			for (int jjj = 0; jjj < localMat.rows; jjj++) {
				freq *= localMat.at<double>(jjj,jjj);
			}
			configProbs.push_back(freq);

			if (freq < 0) printf("%s << ERROR! freq < 0.\n", __FUNCTION__);
			freqSumm += freq;
		}
		for (int iii = 0; iii < configProbs.size(); iii++) {
			if (freqSumm == 0.0) {
				configProbs.at(iii) = 0.0;
			} else configProbs.at(iii) /= freqSumm;
			if (debug) cout << "CFG (" << iii << ") probability = " << configProbs.at(iii) << endl;
		}

		// H: Combine configurations that assign the search identity to the same blob
		vector<double> finalBlobProbs;
		blob_indices[aaa] = -1;
		bestProbability = -1.0;
		int eqProbs = 1;
		for (int iii = 0; iii < blob_count; iii++) {
			double blobProb = 0.0;
			for (int jjj = 0; jjj < configProbs.size(); jjj++) {
				for (int kkk = 0; kkk < identicalSearchObjectIndices.size(); kkk++) {
					if (possibleConfigs.at(jjj).at<double>(iii, identicalSearchObjectIndices.at(kkk)) == 1) {
						blobProb += configProbs.at(jjj);
					}
				}
			}
			if (blobProb >= bestProbability) {

				if (blobProb == bestProbability) { // Case where the probability of this blob is the same as the best so far
					eqProbs++;
					int rand_num = rand() % eqProbs;
					if (rand_num == 0) { // Only change the best index in a fraction of cases
						bestProbability = blobProb;
						blob_indices[aaa] = iii;
					}
				} else {
					eqProbs = 1;
					bestProbability = blobProb;
					blob_indices[aaa] = iii;
				}
			}
			finalBlobProbs.push_back(blobProb);
			if (debug) cout << "Probability that blob (" << iii << ") has class (" << objectNames.at(searchObjectIndex) << ") = " << blobProb << endl;
		}
		if (debug) cout << endl;

		blob_confidences[aaa] = bestProbability; // min(1.0, pow(2.0*bestProbability - 1.0, 0.2)/2.0 + 0.55);

		if (confidence > probability_thresholds[aaa]) break;
	}

	

	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		if ((blob_indices[aaa] != -1) && (blob_confidences[aaa] > probability_thresholds[aaa])) {
			confidence = blob_confidences[aaa];
			absoluteBestBlobIndex = blob_indices[aaa];
			break;
		}
		if (blob_indices[0] != blob_indices[1]) {
			int a = 1;
		}
	}

	if (absoluteBestBlobIndex == -1) {
		for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
			if (blob_indices[aaa] != -1) {
				confidence = blob_confidences[aaa];
				absoluteBestBlobIndex = blob_indices[aaa];
				break;
			}
		}
	}

	blob_confidences[MATCHING_METHODS_COUNT] = confidence;
	blob_indices[MATCHING_METHODS_COUNT] = absoluteBestBlobIndex;
	
	//confidence = bestProbability;
	//confidence = min(1.0, pow(2.0*bestProbability - 1.0, 0.2)/2.0 + 0.55);
	//confidence = max(bestProbability, oldBestBlobProb);
	//confidence = 1.0;

	if (debug) {
		//imshow("a", localPriorProbsCombined);
		//cv::waitKey(0);
	}

	return absoluteBestBlobIndex;
}

void imset::extractCurrentContours(){
	//look at mask, extract contour/s, find largest if many, copy largest into currentContour
	//Later, save contour to file, permanent part of library, use for matching

	for (int iii = 0; iii < blobs.size(); iii++) {

		cv::Mat temp1 = generateSpecificMask(inputMask, iii);
		//findContours modifies source image. Use copyTo to preserve original mask. If not necessary, remove.

		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;

		cv::findContours(temp1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, cv::Point(0, 0));	
		double contArea=-1.0;
		int contourIndex=0;
		if(contours.size()>1){
			contourIndex=-1;
			for(int i = 0; i<contours.size(); i++){
				double area = cv::contourArea(contours[i], false);
				if(area > contArea){
					contArea = area;
					contourIndex = i;
				}
			}
		}
		else{
			contArea = cv::contourArea(contours[contourIndex], false);
		}

		if (0) {
			cv::Mat drawing = cv::Mat::zeros( temp1.size(), CV_8UC3 );
			cv::Scalar color = cv::Scalar( 0, 0, 255 );		//BGR order
			drawContours(drawing, contours, contourIndex, color, 2, 8, hierarchy, 0, cv::Point());
			imshow("test", drawing);
		}

		//cv::waitKey(0);
		blobs[iii].contour = contours[contourIndex];

		/*
		if(libraryMode == 0){
			string filepath = libraryImageFolder + "/" + libraryObjectFolders.at(currentObj) +"/rgbd/contours/";
			if ( access( filepath.c_str(), 0 ) != 0 ){
				CreateDirectory(filepath.c_str() ,NULL);
			}
			std::string::size_type pos = (library_maskNames.at(currentObj).at(currentMask)).find("mask");
			string filename = libraryImageFolder + "/" + libraryObjectFolders.at(currentObj) + "/rgbd/contours/" + (library_maskNames.at(currentObj).at(currentMask)).replace(pos,4,"contour");
			cv::imwrite(filename, drawing);
		}
		*/

		//Below: Interesting code. Bounding box, center of contour, minEnclosingCircle. May be useful for contour matching.

		//	vector<vector<cv::Point> > contours_poly( contours.size() );
		//	vector<cv::Rect> boundRect( contours.size() );
		//	vector<cv::Point2f>center( contours.size() );
		//	vector<float>Distance( contours.size() );
		//	vector<float>radius( contours.size() );

		//	cv::Mat drawing = cv::Mat::zeros(toFix.rows, toFix.cols, CV_8U);
		//	int num_object = 0;
		//	for( int i = 0; i < contours.size(); i++ ){
		//		approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );

		//			// To get rid of the smaller object and the outer rectangle created
		//			//because of the additional mask image we enforce a lower limit on area 
		//			//to remove noise and an upper limit to remove the outer border.    

		//		if (contourArea(contours_poly[i])>(toFix.rows*toFix.cols/10000) && contourArea(contours_poly[i])<toFix.rows*toFix.cols*0.9){
		//			boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
		//			cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
		//			circle(drawing, center[i], (int)radius[i], cv::Scalar(255,255,255), 2, 8, 0);
		//			rectangle(drawing, boundRect[i], cv::Scalar(255,255,255),2,8,0);
		//			//num_object++;
		//		}
		//	}
	}

}

void objectIdentifier::displayMatch(int guessed_index) {

	testImage.generateFeatureImages(true);
	testImage.showFeatureImages(guessed_index, "image_blob");

	if (bestMatchingIndex == -1) return;
	
	int object_index = lib.findObjectIndex(searchName);
	lib.loadImage(object_index, bestMatchingIndex);
	imset a = lib.images->at(object_index).at(bestMatchingIndex);
	a.detectFeatures();
	a.classifyFeatures();
	a.generateFeatureImages();
	a.showFeatureImages(0, "matched_blob");

	cv::waitKey(0);
	
}

void objectIdentifier::identifyObject(const vector<std::string>& objectNames, int index, int numViews, int& blob_index, double& confidence, vector<vector<double> >& viewRotations, vector<std::string>& versionNames, double& preConfidence, bool debug, bool avoid_best, bool evaluation_mode) {

	bool matchScoresPresent = (blobMatchScores[0].size() != 0) ? true : false;

    viewRotations.clear();

	searchName = objectNames.at(index);

	if (!matchScoresPresent) {

		if (debug) cout << "[" << __FUNCTION__ << "] Match scores to be generated on the fly" << endl;

		if (testImage.inputMask.rows == 0) testImage.inputMask = generateMask(testImage.inputImage, int(objectNames.size()));
		testImage.describe();

		if (debug) {

			for (int iii = 0; iii < objectNames.size(); iii++) {
				stringstream ss;
				ss << resultsOutputDir << "/blob_images/im_" << setfill('0') << setw(6) << counter << "_" << iii << ".jpg";
				testImage.generateFeatureImages();
				testImage.saveFeatureImage(ss.str(), iii);
			}
			
		}

		vector<string> emptyVec;
		for (int iii = 0; iii < objectNames.size(); iii++) {
			lib.fullyMatch(testImage.blobs[iii], evaluation_mode ? emptyVec : objectNames, debug);
			lib.retrieveMatchingResults(blobMatchScores);
			if (debug) {
				cout << "[" << __FUNCTION__ << "] iii = (" << iii << ") blobMatchScores[0].size() = " << blobMatchScores[0].size() << endl;
				cout << "[" << __FUNCTION__ << "] blobMatchScores[0].at(" << iii << ").size() = " << blobMatchScores[0].at(iii).size() << endl;
				cout << "[" << __FUNCTION__ << "] blobMatchScores[0].at(" << iii << ").at(7).size() = " << blobMatchScores[0].at(iii).at(7).size() << endl;
			}
		}
	}

	for (int iii = 0; iii < objectNames.size(); iii++) {

		if (debug) cout << "[" << __FUNCTION__ << "] iii = " << iii << endl;

		int temp_bestMatchingIndex = 0;
		double temp_currentConfidence = 0.0;
		int temp_currentBlob = 0;
        vector<string> temp_currentViewLabels, temp_currentVersionNames;
        vector<vector<double> > temp_currentViewRotations;
		double temp_currentPreConfidence = 0.0;

		vector<int> true_indices;
		if (evaluation_mode) {
			for (int jjj = 0; jjj < objectNames.size(); jjj++) {
				if (objectNames.at(iii) == objectNames.at(jjj)) true_indices.push_back(jjj);
			}
		}

		if (!evaluation_mode && (iii != index)) continue; // Unless in test all mode, skip all tests except the actual target object

		// Find which test image blob matches the desired object
		if (debug) cout << "[" << __FUNCTION__ << "] About to call findMatchingIdentity()" << endl;

		int currentBlob_array[MATCHING_METHODS_COUNT+1];
		double currentConfidence_array[MATCHING_METHODS_COUNT+1];
		temp_currentBlob = findMatchingIdentity(objectNames, temp_currentConfidence, iii, debug, currentBlob_array, currentConfidence_array);
		
		if (debug) cout << "[" << __FUNCTION__ << "] Called." << endl;
		if (temp_currentBlob == -1) {
			cout << "ERROR! No matching identity was found from the provided image." << endl;
		}
		if (evaluation_mode) {
			if (!matchScoresPresent && ((iii == 0) && ((objectNames.at(temp_currentBlob) != objectNames.at(iii)) || debug))) {
				generateFailureImage(objectNames, objectNames.at(temp_currentBlob) == objectNames.at(iii));
			}
			
			updateIdentificationMatrices(objectNames, true_indices, currentBlob_array, currentConfidence_array, debug);
		}

		// Then find best matching view
		int bestMatchingObject = lib.findObjectIndex(objectNames.at(iii));
		if (debug) cout << "[" << __FUNCTION__ << "] About to find matching view." << endl;
        findMatchingViews(temp_currentBlob, bestMatchingObject, temp_currentViewLabels, temp_currentViewRotations, temp_currentPreConfidence, numViews, debug, avoid_best);
		if (debug) cout << "[" << __FUNCTION__ << "] Done." << endl;
		// if (evaluation_mode) processViewResults();

		// Determine version names based on view labels
		// (Currently just dummy - uses "standard" for all)
		for (int jjj = 0; jjj < temp_currentViewLabels.size(); jjj++) temp_currentVersionNames.push_back("standard");

		if (iii == index) { // Only copy results to output if it's the real test
			currentBlob = temp_currentBlob;
			currentConfidence = temp_currentConfidence;
            currentViewLabels = temp_currentViewLabels;
            currentViewRotations = temp_currentViewRotations;
			currentVersionNames = temp_currentVersionNames;
			currentPreConfidence = temp_currentPreConfidence;
		}

		if (evaluation_mode) {
			testResult tR;
			
			for (int jjj = 0; jjj < objectNames.size(); jjj++) {
				if (jjj == iii) {
					tR.true_id = lib.findObjectIndex(objectNames.at(jjj));
				} else tR.false_ids.push_back(lib.findObjectIndex(objectNames.at(jjj)));
			}

			tR.onlineConfidence = temp_currentConfidence;
			// tR.result = 

		}
		
	}
	
	blob_index = currentBlob;
	confidence = currentConfidence;
    viewRotations = currentViewRotations;
	versionNames = currentVersionNames;
	preConfidence = currentPreConfidence;

	//cout << "Confidence = " << confidence << endl;

	counter++;
}

void objectIdentifier::outputResultSummary(string realObject, string realName) { 

	int est_obj_id = lib.findObjectIndex(realObject);

	string matchedName = currentViewLabels.at(0);

	std::printf("%s << Blob (%d) identified as (%s/%s) with confidence of (%f); view label (%s) with confidence (%f)\n", __FUNCTION__, currentBlob, realObject.c_str(), realName.c_str(), currentConfidence, matchedName.c_str(), currentPreConfidence); 
}

void objectLibrary::initializeOutputFile() {
	string summaryFileAddress = yml_directory + "../library_summary.txt";
    libraryGenSummary.open (summaryFileAddress.c_str());
}

bool objectLibrary::prepareLibraryDirectories() {

    makeDirectory(yml_directory);

    for (int iii = 0; iii < objectFolders.size(); iii++) {
        string new_dir = yml_directory + "/" + objectFolders.at(iii);
        makeDirectory (new_dir);
    }

	initializeOutputFile();

	return true;
}

void objectLibrary::saveFeatureLibraryComponent(int obj_idx, int img_idx) {

	size_t index = 0;
	index = images->at(obj_idx).at(img_idx).rgbName.find(".", index);
	string outputName = images->at(obj_idx).at(img_idx).rgbName;
	outputName.replace(index, 4, ".yml");

	char new_dir[256];
	sprintf(new_dir, "%s/%s", yml_directory.c_str(), objectFolders.at(obj_idx).c_str());

    string descriptor_file = yml_directory + "/" + objectFolders.at(obj_idx) + "/" + outputName;
    //std::printf("%s << Saving YML data to (%s)\n", __FUNCTION__, descriptor_file.c_str());

	cv::FileStorage fs(descriptor_file, cv::FileStorage::WRITE);
	fs << "foreground_descs" << images->at(obj_idx).at(img_idx).blobs[0].foregroundDescs;
	fs << "edge_descs" << images->at(obj_idx).at(img_idx).blobs[0].edgeDescs;
	fs << "colour_hist" << images->at(obj_idx).at(img_idx).blobs[0].colourHist;
	fs << "contour" << images->at(obj_idx).at(img_idx).blobs[0].contour;
	fs << "rotation" << images->at(obj_idx).at(img_idx).blobs[0].rotation;
	fs << "width_mm" << images->at(obj_idx).at(img_idx).blobs[0].width;
	fs << "height_mm" << images->at(obj_idx).at(img_idx).blobs[0].height;

	fs.release();

	libraryGenSummary << obj_idx << " " << img_idx << " " << images->at(obj_idx).at(img_idx).blobs[0].foregroundDescs.rows << " " << images->at(obj_idx).at(img_idx).blobs[0].edgeDescs.rows << endl;

}

void objectLibrary::loadTransformation(int obj_idx, int img_idx) {

	size_t index = 0;
	index = images->at(obj_idx).at(img_idx).rgbName.find(".", index);
	string xfName = images->at(obj_idx).at(img_idx).rgbName;
	xfName.replace(index, 3, ".xf");
	xfName = xfName.substr(0,xfName.size()-1);

	std::string xfAddress = image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/tfm/" + xfName;

	readXfTransformation(xfAddress, images->at(obj_idx).at(img_idx).transform);
			
	//cout << "transform = " << transform << endl;
}

void objectIdentifier::supplyLatestImage(const cv::Mat& rgbImage, const cv::Mat& maskImage, const cv::Mat& depthImage) { 

	// Clear any stored results that may have been generated or supplied for the previous image
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		lib.blobMatchScores[aaa].clear();
		blobMatchScores[aaa].clear();
	}

	testImage.assign(rgbImage, maskImage, depthImage); 
}

void imageDirectory::loadImage(int obj_idx, int img_idx) {

	if ((images->at(obj_idx).at(img_idx).inputImage.rows > 0) && (images->at(obj_idx).at(img_idx).inputMask.rows > 0)) return; // Image already loaded!

	bool debug = false;

    // std::printf("Loading image (%s)..\n", images->at(obj_idx).at(img_idx).rgbName.c_str());
	//std::printf("%s << Loading rgb image (%s) and mask (%s)\n", __FUNCTION__, images->at(obj_idx).at(img_idx).rgbName.c_str(), images->at(obj_idx).at(img_idx).maskName.c_str());

	string rgbFilename, maskFilename, depthFilename;

	if (directoryFormat == SINGLE_OBJECT_FORMAT) {
		rgbFilename = image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/rgb/" + images->at(obj_idx).at(img_idx).rgbName;
		maskFilename = image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/mask/" + images->at(obj_idx).at(img_idx).maskName;
		depthFilename = image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/depth/" + images->at(obj_idx).at(img_idx).depthName;
	} else if (directoryFormat == MULTI_OBJECT_FORMAT) {
		rgbFilename = image_directory + "/rgb/" + images->at(obj_idx).at(img_idx).rgbName;
		maskFilename = image_directory + "/mask/" + images->at(obj_idx).at(img_idx).maskName;
		depthFilename = image_directory + "/depth/" + images->at(obj_idx).at(img_idx).depthName;
	}

	cv::Mat tempInput, tempMask, tempDepth;
	tempInput = cv::imread(rgbFilename);
	tempMask = cv::imread(maskFilename);
	tempDepth = cv::imread(depthFilename, CV_LOAD_IMAGE_ANYDEPTH);
	//printf("tempDepth.type() = (%d); tempDepth.depth = (%d); type = (%d); depth = (%d)", tempDepth.type(), tempDepth.depth(), CV_16UC1, CV_16U);

	if (tempInput.rows == 0) {
		cout << "ERROR! Unable to load image at <" << rgbFilename << ">" << endl;
		return;
	}

	if (tempMask.rows == 0) {
		cout << "ERROR! Unable to load image at <" << maskFilename << ">" << endl;
		return;
	}

	if (tempDepth.rows == 0) {
		cout << "ERROR! Unable to load image at <" << depthFilename << ">" << endl;
		return;
	}

	cv::Mat finalInput, finalMask, finalDepth;

	if (tempMask.rows != 0) cvtColor(tempMask, finalMask, cv::COLOR_BGR2GRAY);

	if (finalMask.rows == 0) {
		std::printf("%s << ERROR! No mask found! Attempting to generate one..\n", __FUNCTION__);
		int assumedCount = int(images->at(obj_idx).at(img_idx).blobs.size());
		
		if (assumedCount == 0) assumedCount = 3;
			
		finalMask = generateMask(tempInput, assumedCount);
		images->at(obj_idx).at(img_idx).maskName = images->at(obj_idx).at(img_idx).rgbName;
		size_t index = 0;
        index = images->at(obj_idx).at(img_idx).maskName.find(".", index);
        if (index != string::npos) images->at(obj_idx).at(img_idx).maskName.replace(index, 4, ".png");
		if (directoryFormat == SINGLE_OBJECT_FORMAT) {
			makeDirectory(image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/mask/");
			cv::imwrite(image_directory + "/" + objectFolders.at(obj_idx) + "/rgbd/mask/" + images->at(obj_idx).at(img_idx).maskName, finalMask);
		} else {
			makeDirectory(image_directory + "/mask/");
			cv::imwrite(image_directory + "/mask/" + images->at(obj_idx).at(img_idx).maskName, finalMask);
		}
	}

	/*
	int objCount = countObjects(finalMask);

	if (objCount > 3) {
		std::printf("%s << ERROR! More than 3 objects present in image!\n", __FUNCTION__);
		imshow("finalMask", finalMask);
		cv::waitKey(0);
	}
	*/

	// if (images->at(obj_idx).at(img_idx).objectCount < 0) images->at(obj_idx).at(img_idx).objectCount = objCount;

	if (directoryFormat == SINGLE_OBJECT_FORMAT) images->at(obj_idx).at(img_idx).blobs[0].objectName = objectFolders.at(obj_idx);

	/*
	if (images->at(obj_idx).at(img_idx).blobs.size() == 1) {
		if (debug) imshow("mask", cropImage(finalMask));
		cleanMask(finalMask, finalMask);
		fillHoles(finalMask, finalMask, 1);
		finalMask = (finalMask > 0);
	}
	*/

	if (debug) imshow("mask", cropImage(finalMask));
	if (debug) cv::waitKey();

	//histogramNormalize(tempInput, finalInput, finalMask, 1, 3);
	tempInput.copyTo(finalInput);

	// Check if depth has been multiplied by 5...
	int multFactor = 5;
	for (int iii = 0; iii < tempDepth.rows; iii++) {
		for (int jjj = 0; jjj < tempDepth.cols; jjj++) {
			if (tempDepth.at<unsigned short>(iii,jjj) % 5 != 0) {
				multFactor = 1;
				break;
			}
		}
		if (multFactor == 1) break;
	}
	finalDepth = tempDepth / multFactor;

	images->at(obj_idx).at(img_idx).assign(finalInput, finalMask, finalDepth);

}
	
void imageDirectory::limitNPAngles(const vector<int>& npVals) {
	allowableElevations.clear();
	for (int iii = 0; iii < npVals.size(); iii++) allowableElevations.push_back(npVals.at(iii));
}

double calculateMatchingScore(const blob& blob_1, const blob& blob_2, int method) {

	double matchingScore = std::numeric_limits<double>::max();

	// Feature-based matching
	if (method == MM_CONTOUR_COMPARISON) { //Contour Matching
		matchingScore = cv::matchShapes(blob_1.contour, blob_2.contour, CV_CONTOURS_MATCH_I1, 0.0);		//(contour1, contour2, method, param) methods: CV_CONTOURS_MATCH_I1, CV_CONTOURS_MATCH_I2, CV_CONTOURS_MATCH_I3. param not supported yet. Leave as 0.0.
	} else if (method == MM_HISTOGRAM_COMPARISON) { //Histogram Matching
		matchingScore = colourHistMatch(blob_1.colourHist, blob_2.colourHist);
	} else if (method == MM_DIMENSIONS_COMPARISON) { //Histogram Matching
		matchingScore = dimensionsMatch(blob_1.width, blob_1.height, blob_2.width, blob_2.height);
	} else if (method == MM_FEATURES_COMPARISON) {
		matchingScore = featuresMatch(blob_1.foregroundDescs, blob_1.edgeDescs, blob_2.foregroundDescs, blob_2.edgeDescs);
	} 

	// if (matchingScore == 0.0) printf("%s << WARNING! Achieved perfect match..\n", __FUNCTION__);
	return matchingScore;
}

void objectLibrary::calculateIdentificationScores(const vector<vector<double> >& matchingScores, vector<double>& identificationScores) {

	// top matching score
	// 80th percentile matching score
	// average matching score
	// !not using at present! percentage of top 20% matches of this identity

	for (int iii = 0; iii < matchingScores.size(); iii++) { // For each library object
		double topMatchScore = getPercentileValue(matchingScores.at(iii), 0.0);
		//double perc80Score = getPercentileValue(matchingScores.at(iii), 0.2);
		//double averageMatchScore = accumulate(matchingScores.at(iii).begin(), matchingScores.at(iii).end(), 0.0) / double(matchingScores.at(iii).size());

		/*
		vector<double> internalVec;

		internalVec.push_back(topMatchScore);
		internalVec.push_back(perc80Score);
		internalVec.push_back(averageMatchScore);
		*/

		identificationScores.push_back(topMatchScore);
	}


}

void objectLibrary::addDummyObjects(vector<string>& object_names, int num) {
	
	if (num > objectFolders.size()) return;

	while (object_names.size() < num) {
		int idx = rand() % objectFolders.size();

		bool alreadyAdded = false;
		for (int iii = 0; iii < object_names.size(); iii++) {
			if (objectFolders.at(idx) == object_names.at(iii)) alreadyAdded = true;
		}
		if (!alreadyAdded) object_names.push_back(objectFolders.at(idx));
	}
}

void imageDirectory::getIdentities(int obj_idx, int img_idx, vector<string>& object_names, vector<string>& view_names) {
	for (int iii = 0; iii < images->at(obj_idx).at(img_idx).blobs.size(); iii++) {
		string object_name = images->at(obj_idx).at(img_idx).blobs[iii].objectName;
		string view_name = images->at(obj_idx).at(img_idx).blobs[iii].viewName;
		
		size_t index = 0;
		index = view_name.find(".", index);

		if (index != string::npos) view_name = view_name.substr(0, index);

		object_names.push_back(object_name);
		view_names.push_back(view_name);
	}
}

void imageDirectory::retrieveImage(int obj_idx, int img_idx, cv::Mat& img, cv::Mat& mask, cv::Mat& depth) {
	img = images->at(obj_idx).at(img_idx).inputImage;
	mask = images->at(obj_idx).at(img_idx).inputMask;
	depth = images->at(obj_idx).at(img_idx).inputDepth;
}

bool objectLibrary::estimateBestMatchFromName(const blob& input_blob, int& est_obj_id, int& est_img_id) {

	est_obj_id = -1;
	est_img_id = -1;

	for (int iii = 0; iii < objectFolders.size(); iii++) {
		if (input_blob.objectName == objectFolders.at(iii)) {
			est_obj_id = iii;
			for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {
				if (input_blob.viewName == images->at(iii).at(jjj).blobs[0].viewName) {
					est_img_id = jjj;
					break;
				}
			}
			break;
		}
	}

	if ((est_obj_id == -1) || (est_img_id == -1)) {
		std::printf("%s << Error! The test image object identity was not found in the library.\n", __FUNCTION__);
		return false;
	} else return true;
	
}

void imageDirectory::saveImage(char *dir, int obj_idx, int img_idx, cv::Mat& im, string ext) {

    makeDirectory(string(dir));

	char new_dir[256];
	sprintf(new_dir, "%s/%s", dir, objectFolders.at(obj_idx).c_str());

    makeDirectory(string(new_dir));

	size_t index = 0;
	index = images->at(obj_idx).at(img_idx).rgbName.find(".", index);
	string outputName = images->at(obj_idx).at(img_idx).rgbName;
	outputName.insert(index, ext);
	cv::imwrite(string(new_dir) + "/" + outputName, im);

}

void imset::showImage() {
	imshow("rgb", inputImage);
	cv::waitKey(1);
	imshow("mask", inputMask);
}

void imset::detectFeatures(int desiredFeatures) {
	
	detectedPoints.clear();

	// Find extents of object using mask
	int extents[4];
	findExtents(inputMask, extents);
	// std::printf("%s << Max extents of mask = (%d, %d) to (%d, %d)\n", __FUNCTION__, extents[0], extents[1], extents[2], extents[3]);

	cvtColor(inputImage, grayIm, cv::COLOR_RGB2GRAY);

	cv::resize(grayIm, expandedIm, cv::Size(FEATURES_SCALE_FACTOR*grayIm.cols, FEATURES_SCALE_FACTOR*grayIm.rows));
	cv::resize(inputMask, expandedMask, cv::Size(FEATURES_SCALE_FACTOR*inputMask.cols, FEATURES_SCALE_FACTOR*inputMask.rows));

	// ORB for detection and description
	/*
	if (desiredFeatures != DEFAULT_DESIRED_FEATURES) detector = new cv::ORB(desiredFeatures);
	(*detector)(grayIm, mask, detectedPoints, descs);
	*/

	cv::StarFeatureDetector detector;

	cv::Mat combinedMask = generateCombinedMask(expandedMask);

	detector(expandedIm, detectedPoints);

	//detector(expandedIm, combinedMask, detectedPoints, descs);
	sort(detectedPoints.begin(), detectedPoints.end(), compareByResponse); // This sorting will invalidate the existing descriptors! If they are replaced, that is OK.

	for (int iii = 0; iii < detectedPoints.size(); iii++) {
		detectedPoints.at(iii).pt.x /= float(FEATURES_SCALE_FACTOR);
		detectedPoints.at(iii).pt.y /= float(FEATURES_SCALE_FACTOR);
		detectedPoints.at(iii).size /= float(FEATURES_SCALE_FACTOR);
	}

	// if (detectedPoints.size() < (desiredFeatures/2)) std::printf("%s << Only detected (%d) points for (%d) blobs!\n", __FUNCTION__, int(detectedPoints.size()), objectCount);

}

bool objectIdentifier::outputIdentificationResultsSummary(const vector<string>& objects, int true_index, int result_index) {
	std::printf("Blob (%s) identification: %s\n", objects.at(true_index).substr(0, min(12, int(objects.at(true_index).size()))).c_str(), (true_index == result_index) ? "SUCCESS" : "FAILURE");
	return (true_index == result_index);
}

void objectIdentifier::updateIdentificationMatrices(const vector<string>& objects, const vector<int>& true_indices, int *result_index, double *confidence, bool debug) {

	for (int aaa = 0; aaa <= MATCHING_METHODS_COUNT; aaa++) {
		if (idSuccess[aaa].rows == 0) idSuccess[aaa] = cv::Mat::zeros(int(lib.images->size()), int(lib.images->size()), CV_64FC1);
		if (idFailure[aaa].rows == 0) idFailure[aaa] = cv::Mat::zeros(int(lib.images->size()), int(lib.images->size()), CV_64FC1);
		if (idPriorProb[aaa].rows == 0) idPriorProb[aaa] = cv::Mat::zeros(int(lib.images->size()), int(lib.images->size()), CV_64FC1);
	}

	if (idPriorProb3[MATCHING_METHODS_COUNT] == NULL) {
		int sizes[3];
		sizes[0] = int(lib.images->size());
		sizes[1] = int(lib.images->size());
		sizes[2] = int(lib.images->size());
		//int sizes[] = { 100, 100, 100 };
		for (int aaa = 0; aaa <= MATCHING_METHODS_COUNT; aaa++) {
			idSuccess3[aaa] = new cv::Mat(3, sizes, CV_64FC1, cv::Scalar(0));
			idFailure3[aaa] = new cv::Mat(3, sizes, CV_64FC1, cv::Scalar(0));
			idPriorProb3[aaa] = new cv::Mat(3, sizes, CV_64FC1, cv::Scalar(0));
		}
	}

	// Find object indices
	int trueLibraryIndex;
	vector<int> otherLibraryIndices;
	for (int iii = 0; iii < objects.size(); iii++) {
		if (iii == true_indices.at(0)) {
			trueLibraryIndex = lib.findObjectIndex(objects.at(iii));
		} else {
			otherLibraryIndices.push_back(lib.findObjectIndex(objects.at(iii)));
		}
	}

	for (int aaa = 0; aaa <= MATCHING_METHODS_COUNT; aaa++) {
		bool isCorrect = false;

		for (int iii = 0; iii < true_indices.size(); iii++) {
			if (result_index[aaa] == true_indices.at(iii)) isCorrect = true;
		}

		if (objects.size() == 2) {
			if (isCorrect) {
				idSuccess[aaa].at<double>(trueLibraryIndex, otherLibraryIndices.at(0)) += 1.0;
				if (aaa == MATCHING_METHODS_COUNT) confidenceRecord[0].push_back(1.0);
			} else {
				idFailure[aaa].at<double>(trueLibraryIndex, otherLibraryIndices.at(0)) += 1.0;
				if (aaa == MATCHING_METHODS_COUNT) confidenceRecord[0].push_back(0.0);
			}
			if (aaa == MATCHING_METHODS_COUNT) confidenceRecord[1].push_back(confidence[aaa]);
		} else if (objects.size() == 3) {
			if (isCorrect) {
				idSuccess3[aaa]->at<double>(trueLibraryIndex, otherLibraryIndices.at(0), otherLibraryIndices.at(1)) += 1.0;
				if (aaa == MATCHING_METHODS_COUNT) confidenceRecord3[0].push_back(1.0);
			} else {
				idFailure3[aaa]->at<double>(trueLibraryIndex, otherLibraryIndices.at(0), otherLibraryIndices.at(1)) += 1.0;
				if (aaa == MATCHING_METHODS_COUNT) confidenceRecord3[0].push_back(0.0);
			}
			if (aaa == MATCHING_METHODS_COUNT) confidenceRecord[3].push_back(confidence[aaa]);
		}
	
		idPriorProb[aaa] = idSuccess[aaa] / (idSuccess[aaa] + idFailure[aaa]);
		*(idPriorProb3[aaa]) = *(idSuccess3[aaa]) / (*(idSuccess3[aaa]) + *(idFailure3[aaa]));

		for (int iii = 0; iii < idPriorProb[aaa].rows; iii++) {
			for (int jjj = 0; jjj < idPriorProb[aaa].rows; jjj++) {
				if ((idSuccess[aaa].at<double>(iii,jjj) == 0.0) && (idFailure[aaa].at<double>(iii,jjj) == 0.0)) idPriorProb[aaa].at<double>(iii,jjj) = -1.0;
				if ((idSuccess[aaa].at<double>(iii,jjj) > 0.0) && (idFailure[aaa].at<double>(iii,jjj) == 0.0)) idPriorProb[aaa].at<double>(iii,jjj) = 1.0;
				for (int kkk = 0; kkk < idPriorProb[aaa].rows; kkk++) {
					if ((idSuccess3[aaa]->at<double>(iii,jjj,kkk) == 0.0) && (idFailure3[aaa]->at<double>(iii,jjj,kkk) == 0.0)) idPriorProb3[aaa]->at<double>(iii,jjj,kkk) = -1.0;
					if ((idSuccess3[aaa]->at<double>(iii,jjj,kkk) > 0.0) && (idFailure3[aaa]->at<double>(iii,jjj,kkk) == 0.0)) idPriorProb3[aaa]->at<double>(iii,jjj,kkk) = 1.0;
				}
			}
		}
	}

}

void imset::generateFeatureImages(bool use_mask) {

	for (int aaa = 0; aaa < blobs.size(); aaa++) {
		cv::Mat dispMat,  grayColIm, maskedIm, uncroppedIm, specificMask;
		specificMask = generateSpecificMask(inputMask, aaa);
		cvtColor(inputImage, grayIm, cv::COLOR_RGB2GRAY);
		(use_mask) ? grayIm.copyTo(maskedIm, specificMask) : grayIm.copyTo(maskedIm);
		cvtColor(maskedIm, grayColIm, cv::COLOR_GRAY2RGB);
		displayKeyPoints(grayColIm, blobs[aaa].foregroundFeatures, blobs[aaa].edgeFeatures, uncroppedIm);

		blobs[aaa].featuresIm = cropImageFixed(uncroppedIm, specificMask, 200, false);
		blobs[aaa].croppedBlob = cropImageFixed(inputImage, specificMask, 200, false);

		/*
		blobs[aaa].featuresIm = cv::Mat(featuresSide.rows, featuresSide.cols*2, featuresSide.type());

		for (int iii = 0; iii < croppedOrig.rows; iii++) {
			for (int jjj = 0; jjj < croppedOrig.cols; jjj++) {
				for (int kkk = 0; kkk < 3; kkk++) blobs[aaa].featuresIm.at<cv::Vec3b>(iii,jjj)[kkk] = croppedOrig.at<cv::Vec3b>(iii,jjj)[kkk];
			}
		}

		for (int iii = 0; iii < featuresSide.rows; iii++) {
			for (int jjj = 0; jjj < featuresSide.cols; jjj++) {
				for (int kkk = 0; kkk < 3; kkk++) blobs[aaa].featuresIm.at<cv::Vec3b>(iii,jjj+featuresSide.cols)[kkk] = featuresSide.at<cv::Vec3b>(iii,jjj)[kkk];
			}
		}
		*/
	}
	

}

void imset::describeFeatures() {
	cv::FREAK extractor(false, true);

	for (int iii = 0; iii < blobs.size(); iii++) {

		for (int jjj = 0; jjj < blobs[iii].foregroundFeatures.size(); jjj++) {
			blobs[iii].foregroundFeatures.at(jjj).pt.x *= float(FEATURES_SCALE_FACTOR);
			blobs[iii].foregroundFeatures.at(jjj).pt.y *= float(FEATURES_SCALE_FACTOR);
			blobs[iii].foregroundFeatures.at(jjj).size *= float(FEATURES_SCALE_FACTOR);
		}

		blobs[iii].foregroundDescs = cv::Mat();
		extractor.compute(expandedIm, blobs[iii].foregroundFeatures, blobs[iii].foregroundDescs );

		for (int jjj = 0; jjj < blobs[iii].foregroundFeatures.size(); jjj++) {
			blobs[iii].foregroundFeatures.at(jjj).pt.x /= float(FEATURES_SCALE_FACTOR);
			blobs[iii].foregroundFeatures.at(jjj).pt.y /= float(FEATURES_SCALE_FACTOR);
			blobs[iii].foregroundFeatures.at(jjj).size /= float(FEATURES_SCALE_FACTOR);
		}

		for (int jjj = 0; jjj < blobs[iii].edgeFeatures.size(); jjj++) {
			blobs[iii].edgeFeatures.at(jjj).pt.x *= float(FEATURES_SCALE_FACTOR);
			blobs[iii].edgeFeatures.at(jjj).pt.y *= float(FEATURES_SCALE_FACTOR);
			blobs[iii].edgeFeatures.at(jjj).size *= float(FEATURES_SCALE_FACTOR);
		}

		blobs[iii].edgeDescs = cv::Mat();
		extractor.compute(expandedIm, blobs[iii].edgeFeatures, blobs[iii].edgeDescs );

		for (int jjj = 0; jjj < blobs[iii].edgeFeatures.size(); jjj++) {
			blobs[iii].edgeFeatures.at(jjj).pt.x /= float(FEATURES_SCALE_FACTOR);
			blobs[iii].edgeFeatures.at(jjj).pt.y /= float(FEATURES_SCALE_FACTOR);
			blobs[iii].edgeFeatures.at(jjj).size /= float(FEATURES_SCALE_FACTOR);
		}
	}

}

void imset::classifyFeatures() {

	for (int iii = 0; iii < blobs.size(); iii++) {

		cv::Mat specificMask = generateSpecificMask(inputMask, iii);
		std::vector<cv::Point> dilatedEdgePixels, erodedEdgePixels;

		// Dilate mask for feature classification to accept mostly-interior features as interior
		cv::Mat dilatedMask, erodedMask;
		cv::dilate(specificMask, dilatedMask, DEFAULT_STRUCTURING_UNIT);
		cv::erode(specificMask, erodedMask, DEFAULT_STRUCTURING_UNIT);

		determineObjectEdgePixels(dilatedMask, dilatedEdgePixels);
		determineObjectEdgePixels(erodedMask, erodedEdgePixels);

		blobs[iii].foregroundFeatures.clear();
		blobs[iii].edgeFeatures.clear();

		for (int kkk = 0; kkk < detectedPoints.size(); kkk++) {

			int classification = 0;
			cv::Point center = detectedPoints.at(kkk).pt;

			if (doesFeatureIntersect(dilatedEdgePixels, detectedPoints.at(kkk))) {
				if (doesFeatureIntersect(erodedEdgePixels, detectedPoints.at(kkk))) {
					blobs[iii].edgeFeatures.push_back(detectedPoints.at(kkk));
				}
			} else if (dilatedMask.at<unsigned char>(center.y, center.x) != 0) {
				blobs[iii].foregroundFeatures.push_back(detectedPoints.at(kkk));
			}
		}

		if (blobs[iii].edgeFeatures.size() > FEATURES_PER_TYPE) blobs[iii].edgeFeatures.erase(blobs[iii].edgeFeatures.begin(), blobs[iii].edgeFeatures.end()-FEATURES_PER_TYPE);
		if (blobs[iii].foregroundFeatures.size() > FEATURES_PER_TYPE) blobs[iii].foregroundFeatures.erase(blobs[iii].foregroundFeatures.begin(), blobs[iii].foregroundFeatures.end()-FEATURES_PER_TYPE);

	}

}

void imset::measureDimensions() {
	for (int iii = 0; iii < blobs.size(); iii++) {
		cv::Mat specificMask = generateSpecificMask(inputMask, iii);

		double summedDist = 0.0;
		int topRow = specificMask.rows, botRow = 0, leftCol = specificMask.cols, rightCol = 0;

		int maskCount = 0, pixCount = 0;
		for (int aaa = 0; aaa < specificMask.rows; aaa++) {
			for (int bbb = 0; bbb < specificMask.cols; bbb++) {
				if (specificMask.at<unsigned char>(aaa,bbb) > 0) {

					if (aaa < topRow) topRow = aaa;
					if (aaa > botRow) botRow = aaa;
					if (bbb < leftCol) leftCol = bbb;
					if (bbb > rightCol) rightCol = bbb;

					maskCount++;

					if (inputDepth.at<unsigned short>(aaa,bbb) != 0) {
						summedDist += double(inputDepth.at<unsigned short>(aaa,bbb));
						pixCount++;
					}
				}
			}
		}

		if (maskCount == 0) {
			cout << "[" << __FUNCTION__ << "] ERROR! No mask available to calculate blob dimensions." << endl;
			return;
		}

		int pixHeight = botRow - topRow;
		int pixWidth = rightCol - leftCol;

		// to deal with appended images
		while (leftCol > 640) leftCol -= 640;
		while (rightCol > 640) rightCol -= 640;

		if (pixCount == 0) {
			blobs[iii].height = 0.0;
			blobs[iii].width = 0.0;
		} else {
			double distanceToObject = summedDist / double(pixCount);

			double topLeft_x = (double(leftCol) - 319.5) * distanceToObject / 525.0; 
			double topLeft_y = (double(topRow) - 239.5) * distanceToObject / 525.0;
			double botRight_x = (double(rightCol) - 319.5) * distanceToObject / 525.0;
			double botRight_y = (double(botRow) - 239.5) * distanceToObject / 525.0;

			// use intrinsics + distance to convert these to height and width in mm
			blobs[iii].height = abs(topLeft_y-botRight_y);
			blobs[iii].width = abs(topLeft_x-botRight_x);
		}

		if ((blobs[iii].height < 0.0) || (blobs[iii].width < 0.0) || (blobs[iii].height > 500.0) || (blobs[iii].width > 500.0)) {
			cout << "[" << __FUNCTION__ << "] ERROR! Blob width a bit questionable..." << endl;
			return;
		}
	}
}

void imset::generateColourHistograms() {

	for (int iii = 0; iii < blobs.size(); iii++) {

		cv::Mat specificMask = generateSpecificMask(inputMask, iii);

		// Erode histogram a bit to reduce risk of background contamination
		cv::Mat erodedMask;
		cv::erode(specificMask, erodedMask, DEFAULT_STRUCTURING_UNIT);

		bool hsvMode = true;

		/// Using 32 bins for each color channel by default
		int bins = DEFAULT_BIN_NUM;

		blobs[iii].colourHist = cv::Mat();

		int dims = -1;
		int *histSize;

		if (hsvMode) {
			cv::Mat hsvImage;
			cvtColor( inputImage, hsvImage, cv::COLOR_BGR2HSV );

			/// Recommended: Using 50 bins for hue and 60 for saturation
			int h_bins = 15; 
			int s_bins = 15;
			histSize = new int[2];
			histSize[0] = h_bins;
			histSize[1] = s_bins;
			dims = 2;

			// hue varies from 0 to 179, saturation from 0 to 255
			float h_ranges[] = { 0, 180 };
			float s_ranges[] = { 0, 256 };

			const float* ranges[] = { h_ranges, s_ranges };

			// Use the o-th and 1-st channels
			int channels[] = { 0, 1 };

			/// Compute the histograms:
			calcHist(&hsvImage, 1, channels, erodedMask, blobs[iii].colourHist, dims, histSize, ranges, true, false);
		
			//normalize(colourHist, colourHist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
			blobs[iii].colourHist /= float(cv::countNonZero(erodedMask));
		} else {
			histSize = new int[3];
			histSize[0] = bins;
			histSize[1] = bins;
			histSize[2] = bins;
			dims = 3;

			// colors from 0 to 255
			float range[] = { 0, 256 };

			const float* ranges[] = { range, range, range };

			// Use all channels
			int channels[] = { 0, 1, 2 };

			/// Compute the histograms:
			calcHist(&inputImage, 1, channels, erodedMask, blobs[iii].colourHist, dims, histSize, ranges, true, false);
		
			blobs[iii].colourHist /= float(cv::countNonZero(erodedMask));

			//normalize3d(colourHist, colourHist, cv::countNonZero(mask));
			//normalize(colourHist, colourHist, 0, 1, cv::NORM_L1, -1, cv::Mat() );
		}

		/// Draw for each channel
		cv::Mat drawnHistogram;
		drawHistogram(blobs[iii].colourHist, drawnHistogram);
	
		/// Display
		//cv::imshow("input Histogram", drawnHistogram );
		//cv::waitKey(10);

		/*
		cv::Mat colourSegment;
		inputImage.copyTo(colourSegment, mask);
		imshow("colourSegment", colourSegment);
		cv::waitKey(1);
		*/

	}

}

void objectIdentifier::restrictData(const vector<int>& obj_ids, const vector<int>& img_ids) {
	if (obj_ids.size() > 0) lib.limitObjects(obj_ids);
	if (img_ids.size() > 0) lib.limitNPAngles(img_ids);
}

void objectIdentifier::loadConfidencePoints(string params_directory) {
	
	if (params_directory == "") params_directory = resultsOutputDir + "/../params";
	
	cout << "Loading confidence scaling points from (" << params_directory << ")..." << endl;
	
	for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
		identityConfidencePoints[aaa].clear();

		ifstream confidenceFile;
		string filename = params_directory + "/" + confidencePointsFiles[aaa];
		
		confidenceFile.open(filename.c_str());

		if (confidenceFile.is_open()) {
			while (true) {
				double val;
				confidenceFile >> val;
				if (confidenceFile.eof()) break;
				identityConfidencePoints[aaa].push_back(val);
			}
			confidenceFile.close();
		}

		viewRatioPoints[aaa].clear();
		filename = params_directory + "/" + viewRatioPointsFiles[aaa];
		
		confidenceFile.open(filename.c_str());

		if (confidenceFile.is_open()) {
			while (true) {
				double val;
				confidenceFile >> val;
				if (confidenceFile.eof()) break;
				viewRatioPoints[aaa].push_back(val);
			}
			confidenceFile.close();
		}
	}
}

bool objectLibrary::loadExisting(string yml_address, string img_address) {

	images = new vector<vector<imset> >();

    yml_directory = yml_address;
	
	if (!getContents(yml_address, objectFolders)) return false;
    filterContents(objectFolders);
    sort(objectFolders.begin(), objectFolders.end());

    //std::printf("%s << (%d) objects found in reference library..\n", __FUNCTION__, int(objectFolders.size()));

	for (int iii = 0; iii < objectFolders.size(); iii++) {

		std::vector<std::string> descNamesLocal;
		
        string inputDirectory = yml_address + "/" + objectFolders.at(iii);

        if (!getContents(inputDirectory, descNamesLocal)) return false;
        filterContents(descNamesLocal);

		padNames(descNamesLocal);
		sort(descNamesLocal.begin(), descNamesLocal.end());
		unpadNames(descNamesLocal);

		std::vector<imset> objectsVec;

		for (int jjj = 0; jjj < descNamesLocal.size(); jjj++) {
			imset img;
			size_t index = 0;
			index = descNamesLocal.at(jjj).find(".", index);
			
			blob b;
			b.viewName = descNamesLocal.at(jjj).substr(0, index);
			img.blobs.push_back(b);
			objectsVec.push_back(img);
		}

		images->push_back(objectsVec);
		//descNames.push_back(descNamesLocal);
		
	}

    std::printf("Identification YML Library << (%d) objects found, total of (%d) views\n", int(images->size()), int(countViews(images)));

	for (int iii = 0; iii < objectFolders.size(); iii++) {

        //std::printf("Loading object (%03d) of (%03d): <%s>\n", iii+1, int(objectFolders.size()), objectFolders.at(iii).c_str());

		for (int jjj = 0; jjj < images->at(iii).size(); jjj++) {

			char descriptor_file[256];
			sprintf(descriptor_file, "%s/%s/%s.yml", yml_address.c_str(), objectFolders.at(iii).c_str(), images->at(iii).at(jjj).blobs[0].viewName.c_str());

			cv::Mat foregroundDescs, backgroundDescs, refColourHist; // ref_b_hist, ref_g_hist, ref_r_hist;
			//std::vector<int> foregroundIndices, edgeIndices;

			cv::FileStorage fs(descriptor_file, cv::FileStorage::READ);
			fs["foreground_descs"] >> images->at(iii).at(jjj).blobs[0].foregroundDescs;
			fs["edge_descs"] >> images->at(iii).at(jjj).blobs[0].edgeDescs;
			fs["colour_hist"] >> images->at(iii).at(jjj).blobs[0].colourHist;
			fs["contour"] >> images->at(iii).at(jjj).blobs[0].contour;
			fs["rotation"] >> images->at(iii).at(jjj).blobs[0].rotation;
			fs["width_mm"] >> images->at(iii).at(jjj).blobs[0].width;
			fs["height_mm"] >> images->at(iii).at(jjj).blobs[0].height;
			fs.release();
		}
		
	}

	if (img_address.size() > 0) {
		vector<string> img_objectFolders;
		if (!getContents(img_address, img_objectFolders)) return false;
		filterContents(img_objectFolders);

		if (objectFolders.size() != img_objectFolders.size()) {
			std::printf("Number of directories in the raw library image directory != that of the YML directory. Assuming non-correspondence.\n");
		} else {
			setImageDirectory(img_address);
			loadImageLocations();
		}
	}

	return true;

}
