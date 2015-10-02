#include "identification.h"

static bool debugMode = false;
static bool preMatching = true;
static bool tripletTesting = false;

// Set this as true only if you want to avoid the best matching registered view. This is useful if the test image is in the library, and you want to make sure that the 2nd best view is nearby.
static bool avoidFlag = false;

int main(int argc, char *argv[]) {

	std::printf("Loading parameters..\n");

    string yml_data_address = get_address("yml_data_address.txt", "yml_library/yml_library");
	std::printf("YML library directory:  %s\n", yml_data_address.substr((yml_data_address.find_last_of("/\\")==string::npos) ? 0 : yml_data_address.find_last_of("/\\")+1, yml_data_address.size()).c_str());

    string raw_data_address = get_address("raw_data_address.txt", "image_library/image_library");
	std::printf("Raw image directory:    %s\n", raw_data_address.substr((raw_data_address.find_last_of("/\\")==string::npos) ? 0 : raw_data_address.find_last_of("/\\")+1, raw_data_address.size()).c_str());

    string test_data_address = get_address("test_data_address.txt", "test_data/test_data");
	std::printf("Test image directory:   %s\n\n", test_data_address.substr((test_data_address.find_last_of("/\\")==string::npos) ? 0 : test_data_address.find_last_of("/\\")+1, test_data_address.size()).c_str());

	objectIdentifier objI;
	
	std::printf("Loading the library..\n");
	objI.loadLibrary(yml_data_address, raw_data_address);
	std::printf("Library loaded.\n\n");

	imageDirectory testSequence;
	testSequence.setImageDirectory(test_data_address);
	std::printf("Traversing the test sequence..\n");
	testSequence.traverseDirectory();
	size_t blobsToTest = testSequence.getTotalBlobCount();
	std::printf("Test sequence traversed. (%d) images and (%d) blobs in total\n\n", int(testSequence.getTotalImageCount()), int(blobsToTest));

	string development_address = string(_SOURCE_DIRECTORY_) + "/output";
	objI.setOutputDir(development_address);

	vector<string> collectedBlobIdentities, collectedBlobNames;
	vector<int> collectedBlobsIII, collectedBlobsJJJ, collectedBlobsKKK;
	vector<vector<vector<double> > > blobMatchScores[MATCHING_METHODS_COUNT];

	// Go through all images, prep and describe all blobs
	int blobsTested = 0;
	int lastPercentageReported = 0, percentageStep = 5;
	std::printf("Blob %s commencing...\n", (preMatching) ? "matching" : "accumulation");

	vector<int> testDataCoverageVector(25, 0);

	for (int iii = 0; iii < testSequence.getObjectCategoryCount(); iii++) {

		for (int jjj = 0; jjj < testSequence.getImageCount(iii); jjj++) {
					
			vector<string> realObjects, realNames;
			testSequence.getIdentities(iii, jjj, realObjects, realNames);

			testSequence.loadImage(iii, jjj);
			testSequence.images->at(iii).at(jjj).describe();

			cv::Mat rgbImage, maskImage, depthImage;
			testSequence.retrieveImage(iii, jjj, rgbImage, maskImage, depthImage);

			vector<vector<vector<double> > > localBlobMatchScores[MATCHING_METHODS_COUNT];

			objI.lib.clearPreviousResults();

			for (int kkk = 0; kkk < realObjects.size(); kkk++) {

				cv::Mat specificMask = generateSpecificMask(maskImage, kkk);

				blob b = testSequence.images->at(iii).at(jjj).blobs.at(kkk);

				int libraryIndex = objI.lib.findObjectIndex(realObjects.at(kkk));

				if (libraryIndex < 0) {
					cout << "ERROR! Could not find object <" << realObjects.at(kkk) << "> in library." << endl;
					continue;
				}
				testDataCoverageVector.at(libraryIndex) += 1;

				collectedBlobIdentities.push_back(realObjects.at(kkk));
				collectedBlobNames.push_back(realNames.at(kkk));
				collectedBlobsIII.push_back(iii);
				collectedBlobsJJJ.push_back(jjj);
				collectedBlobsKKK.push_back(kkk);

				if (preMatching) {
					vector<string> emptyVec;
					objI.lib.fullyMatch(b, emptyVec);
					objI.lib.retrieveMatchingResults(localBlobMatchScores);
				}
					
				blobsTested++;

				int percentageComplete = percentageStep*int(floor((100.0/double(percentageStep))*double(blobsTested)/double(blobsToTest)));
				if (percentageComplete > lastPercentageReported) {
					std::printf("Blob %s %d%% complete.\n", (preMatching) ? "matching" : "accumulation", percentageComplete);
					lastPercentageReported = percentageComplete;
				}
			}

			if (preMatching) {
			    bool testAborted = false;
                if (!objI.outputConfidenceData(realObjects, localBlobMatchScores)) testAborted = true; 
                else if (!objI.processMatchingResults(realObjects, realNames, localBlobMatchScores)) testAborted = true;

                if (testAborted) {
                    std::printf("\n%s << Exiting... (press any key)\n", __FUNCTION__);
	                cin.get();

#if defined(_WIN32) && defined(_DEBUG) && defined(TerminateProcess)
	                TerminateProcess(GetCurrentProcess(), EXIT_SUCCESS);
#endif
                }

				for (int kkk = 0; kkk < realObjects.size(); kkk++) {
					for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
						blobMatchScores[aaa].push_back(localBlobMatchScores[aaa].at(kkk));
					}
				}
			}
			
		}
		
	}

	ofstream testCoverageStream;
	string testCoverageStream_string = string(development_address) + "/test_data_coverage.txt";
	testCoverageStream.open(testCoverageStream_string.c_str());
	for (int iii = 0; iii < testDataCoverageVector.size(); iii++) {
		string preString = objI.lib.objectFolders.at(iii) + ": ";
		while (preString.size() < 45) preString = preString + " ";
		testCoverageStream << preString << testDataCoverageVector.at(iii) << endl;
	}
	testCoverageStream.close();

	int expectedPairTests = 0, expectedTripletTests = 0;
	for (int iii = int(collectedBlobIdentities.size()-1); iii >= 0; iii--) expectedPairTests += iii;

	for (int iii = 0; iii < int(collectedBlobIdentities.size()-2); iii++) {
		for (int jjj = iii+1; jjj < int(collectedBlobIdentities.size()-1); jjj++) {
			for (int kkk = jjj+1; kkk < int(collectedBlobIdentities.size()); kkk++) {
				expectedTripletTests += 1;
			}
		}
	}

	if (tripletTesting) {
		std::printf("\nAbout to commence (%d) pairwise and (%d) triplet identity tests..\n", expectedPairTests, expectedTripletTests);
	} else std::printf("\nAbout to commence (%d) pairwise identity tests..\n", expectedPairTests);

	int actualPairsTested = 0, actualTripletsTested = 0;
	lastPercentageReported = 0;

	// Here you should generate a whole set of doubles and triples, creating unique mask for each
	for (int aaa = 0; aaa < collectedBlobIdentities.size()-1; aaa++) {
		if (collectedBlobIdentities.size() == 0) break;

		cv::Mat rgbImage, maskImage, depthImage;

		// Need to create synthetic rgbImage and maskImage
		cv::Mat blob1, mask1, depth1;
		if (!preMatching) {
			testSequence.loadImage(collectedBlobsIII.at(aaa), collectedBlobsJJJ.at(aaa));
			cv::Mat metaMask;
			testSequence.retrieveImage(collectedBlobsIII.at(aaa), collectedBlobsJJJ.at(aaa), blob1, metaMask, depth1);

			mask1 = generateSpecificMask(metaMask, collectedBlobsKKK.at(aaa));

			//blob1 = extractObject(rawImage, specificMask);
			//mask1 = extractObject(specificMask, specificMask);
			mask1 = (mask1/255)*80;
		}
		
		vector<vector<vector<double> > > localBlobMatchScores[MATCHING_METHODS_COUNT];
		vector<string> localObjects, localNames;

		localObjects.push_back(collectedBlobIdentities.at(aaa));
		localNames.push_back(collectedBlobNames.at(aaa));

		if (preMatching) { 
			for (int xxx = 0; xxx < MATCHING_METHODS_COUNT; xxx++) {
				localBlobMatchScores[xxx].clear();
				localBlobMatchScores[xxx].push_back(blobMatchScores[xxx].at(aaa)); 
			}
		}

		for (int bbb = aaa+1; bbb < collectedBlobIdentities.size(); bbb++) { // For double test

			if (preMatching) { 
				for (int xxx = 0; xxx < MATCHING_METHODS_COUNT; xxx++) {
					while (localBlobMatchScores[xxx].size() > 1) localBlobMatchScores[xxx].pop_back();
					localBlobMatchScores[xxx].push_back(blobMatchScores[xxx].at(bbb));
				}
			}
					
			while (localObjects.size() > 1) localObjects.pop_back();
			localObjects.push_back(collectedBlobIdentities.at(bbb));
			
			while (localNames.size() > 1) localNames.pop_back();
			localNames.push_back(collectedBlobNames.at(bbb));

			// Append with second object
			cv::Mat blob2, mask2, depth2;
			if (!preMatching) {
				testSequence.loadImage(collectedBlobsIII.at(bbb), collectedBlobsJJJ.at(bbb));
				cv::Mat metaMask;
				testSequence.retrieveImage(collectedBlobsIII.at(bbb), collectedBlobsJJJ.at(bbb), blob2, metaMask, depth2);

				mask2 = generateSpecificMask(metaMask, collectedBlobsKKK.at(bbb));
				
				mask2 = (mask2/255)*160;
				
				rgbImage = appendImage(blob1, blob2);
				maskImage = appendImage(mask1, mask2);
				depthImage = appendImage(depth1, depth2);
			}

			(preMatching) ? objI.supplyPrematchingResults(localBlobMatchScores) : objI.supplyLatestImage(rgbImage, maskImage, depthImage);

			{
				int numViews = DEFAULT_RETURNED_VIEWS; 
				int blob_index;
				double confidence, preConfidence;
                vector<vector<double> > viewRotations;
				vector<string> versionNames;

				int name_index = 0;
				
				actualPairsTested++;

                objI.identifyObject(localObjects, name_index, DEFAULT_RETURNED_VIEWS, blob_index, confidence, viewRotations, versionNames, preConfidence, debugMode, avoidFlag, true);
				
				if (actualPairsTested == 4) objI.saveCurrentBlobMatchScores();

				objI.displayCurrentMatchingResults();
			}

			if (!tripletTesting) {
				
				int percentageComplete = percentageStep*int(floor((100.0/double(percentageStep))*double(actualPairsTested)/double(expectedPairTests)));
				if (percentageComplete > lastPercentageReported) {
					std::printf("Identification %d%% complete.\n", percentageComplete);
					lastPercentageReported = percentageComplete;
				}
				
				continue;
			}

			for (int ccc = bbb+1; ccc < collectedBlobIdentities.size(); ccc++) { // For triple test
				if (ccc >= collectedBlobIdentities.size()) break;

				if (preMatching) { 
					for (int xxx = 0; xxx < MATCHING_METHODS_COUNT; xxx++) {
						while (localBlobMatchScores[xxx].size() > 2) localBlobMatchScores[xxx].pop_back();
						localBlobMatchScores[xxx].push_back(blobMatchScores[xxx].at(ccc));
					}
				}
					
				while (localObjects.size() > 2) localObjects.pop_back();
				localObjects.push_back(collectedBlobIdentities.at(ccc));
			
				while (localNames.size() > 2) localNames.pop_back();
				localNames.push_back(collectedBlobNames.at(ccc));


				// Append with second object
				cv::Mat blob3, mask3, depth3;
				if (!preMatching) {
					testSequence.loadImage(collectedBlobsIII.at(ccc), collectedBlobsJJJ.at(ccc));
					cv::Mat metaMask;
					testSequence.retrieveImage(collectedBlobsIII.at(ccc), collectedBlobsJJJ.at(ccc), blob3, metaMask, depth3);

				
					mask3 = generateSpecificMask(metaMask, collectedBlobsKKK.at(ccc));
				
					mask3 = (mask3/255)*240;
				
					rgbImage = appendImage(blob1, blob2);
					rgbImage = appendImage(rgbImage, blob3);
					maskImage = appendImage(mask1, mask2);
					maskImage = appendImage(maskImage, mask3);
					depthImage = appendImage(depth1, depth2);
					depthImage = appendImage(depthImage, depth3);
				}

				(preMatching) ? objI.supplyPrematchingResults(localBlobMatchScores) : objI.supplyLatestImage(rgbImage, maskImage);

				{
					int numViews = DEFAULT_RETURNED_VIEWS; 
					int blob_index;
					double confidence, preConfidence;
                    vector<vector<double> > viewRotations;
					vector<string> versionNames;

					int name_index = 0;
				
					actualTripletsTested++;
                    objI.identifyObject(localObjects, name_index, DEFAULT_RETURNED_VIEWS, blob_index, confidence, viewRotations, versionNames, preConfidence, debugMode, avoidFlag, true);
					objI.displayCurrentMatchingResults();
				}

				int percentageComplete = percentageStep*int(floor((100.0/double(percentageStep))*double(actualTripletsTested)/double(expectedTripletTests)));
				if (percentageComplete > lastPercentageReported) {
					std::printf("Identification %d%% complete.\n", percentageComplete);
					lastPercentageReported = percentageComplete;
				}

			}

		}
	}

	//std::printf("Identification testing completed. (%d) pairs and (%d) triplets tested.\n", actualPairsTested, actualTripletsTested);

	// View matching assessment
	if (preMatching) {

		std::ofstream viewRatiosFile[MATCHING_METHODS_COUNT];
		for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) {
			stringstream ss;
			ss << development_address << "/view_ratio_data_" << aaa << ".txt";
			viewRatiosFile[aaa].open(ss.str().c_str());
		}

		int actualViewTests = 0;
		std::printf("\nAbout to commence (%d) possible view matching tests..\n", int(collectedBlobIdentities.size()));

		vector<double> viewMatchingScores[MATCHING_METHODS_COUNT], viewMatchingCounts(objI.lib.images->size(), 0.0);
		for (int bbb = 0; bbb < MATCHING_METHODS_COUNT; bbb++) viewMatchingScores[bbb] = vector<double>(objI.lib.images->size(), 0.0);

		vector<double> combined_rank_scores;

		for (int aaa = 0; aaa < collectedBlobIdentities.size(); aaa++) {

			// Look at ordered indices for best match for CORRECT object only
			int trueObjectIndex = objI.lib.findObjectIndex(collectedBlobIdentities.at(aaa));

			vector<double> localBlobMatchScores[MATCHING_METHODS_COUNT];

			if (preMatching) { 
				for (int xxx = 0; xxx < MATCHING_METHODS_COUNT; xxx++) {
					localBlobMatchScores[xxx] = blobMatchScores[xxx].at(aaa).at(trueObjectIndex);
				}
			} else {

				cv::Mat rgbImage, maskImage, depthImage;

				// Need to create synthetic rgbImage and maskImage
				cv::Mat rawImage, rawMask, rawDepth;
				testSequence.loadImage(collectedBlobsIII.at(aaa), collectedBlobsJJJ.at(aaa));
				testSequence.retrieveImage(collectedBlobsIII.at(aaa), collectedBlobsJJJ.at(aaa), rawImage, rawMask, rawDepth);

				cv::Mat specificMask;
				specificMask = generateSpecificMask(rawMask, collectedBlobsKKK.at(aaa));

				rgbImage = extractObject(rawImage, specificMask);
				maskImage = extractObject(specificMask, specificMask);
				maskImage = (maskImage/255)*80;

				depthImage = extractObject(rawDepth, specificMask);

				objI.supplyLatestImage(rgbImage, maskImage, depthImage);

				vector<string> localObjects, localNames;
				localObjects.push_back(collectedBlobIdentities.at(aaa));

				{
					int numViews = DEFAULT_RETURNED_VIEWS; 
					int blob_index;
					double confidence, preConfidence;
                    vector<vector<double> > viewRotations;
					vector<string> versionNames;
			
					int name_index = 0;
				
					actualPairsTested++;
                    objI.identifyObject(localObjects, name_index, DEFAULT_RETURNED_VIEWS, blob_index, confidence, viewRotations, versionNames, preConfidence, debugMode, avoidFlag, false);
					//objI.displayCurrentMatchingResults();

					// retrieve <localBlobMatchScores>
					objI.lib.retrieveSpecificMatchingResults(localBlobMatchScores, trueObjectIndex);
				}
			}

			// Find position of the true name from <collectedBlobNames> (unless no match is found)
			int trueViewIndex = -1;
			for (int iii = 0; iii < objI.lib.images->at(trueObjectIndex).size(); iii++) {
				if (objI.lib.images->at(trueObjectIndex).at(iii).blobs[0].viewName == collectedBlobNames.at(aaa)) {
					trueViewIndex = iii;
					break;
				}
			}
			
			if (trueViewIndex == -1) continue;
			viewMatchingCounts.at(trueObjectIndex)++;
			actualViewTests++;

			if (debugMode) printf("View test (%d); for object (%s) true view (%s)\n", actualViewTests, collectedBlobIdentities.at(aaa).c_str(), collectedBlobNames.at(aaa).c_str());

			cv::Mat viewMatchingMat(MATCHING_METHODS_COUNT, int(viewMatchingScores[0].size()), CV_64FC1);

			vector<score_pair> collected_combined_scores[MATCHING_METHODS_COUNT];

			for (int bbb = 0; bbb < MATCHING_METHODS_COUNT; bbb++) {

				vector<score_pair> combined_scores;
				for (int iii = 0; iii < localBlobMatchScores[bbb].size(); iii++) {
					score_pair sp(localBlobMatchScores[bbb].at(iii), iii);
					combined_scores.push_back(sp);
				}
				std::sort(combined_scores.begin(), combined_scores.end());

				int correctIndexRank = -1;
				for (int iii = 0; iii < combined_scores.size(); iii++) {
					if (combined_scores.at(iii).second == trueViewIndex) {
						correctIndexRank = iii;
					}
				}

				for (int iii = 0; iii < combined_scores.size(); iii++) {
					if (iii == correctIndexRank) continue;
					viewRatiosFile[bbb] << combined_scores.at(correctIndexRank).first/combined_scores.at(iii).first << endl;
				}

				double rankProp = double((combined_scores.size()-1) - correctIndexRank)/double(combined_scores.size()-1);
				viewMatchingScores[bbb].at(trueObjectIndex) += rankProp;

				// double bestScore = *(std::min_element(blobMatchScores[bbb].at(aaa).at(trueObjectIndex).begin(), blobMatchScores[bbb].at(aaa).at(trueObjectIndex).end()));
				// int bestIndex = int(distance(blobMatchScores[bbb].at(aaa).at(trueObjectIndex).begin(),min_element(blobMatchScores[bbb].at(aaa).at(trueObjectIndex).begin(),blobMatchScores[bbb].at(aaa).at(trueObjectIndex).end())));

				if (debugMode) {

					printf("For method [%d]:\n", bbb);
					for (int iii = 0; iii < combined_scores.size(); iii++) {
						printf("#%d view = (%s)\n", iii+1, objI.lib.images->at(trueObjectIndex).at(combined_scores.at(iii).second).blobs[0].viewName.c_str());
					}
					cout << endl;
				}

				for (int iii = 0; iii < viewMatchingScores[bbb].size(); iii++) {
					if (viewMatchingCounts.at(iii) == 0) {
						viewMatchingMat.at<double>(bbb, iii) = -1.0;
					} else viewMatchingMat.at<double>(bbb, iii) = viewMatchingScores[bbb].at(iii) / viewMatchingCounts.at(iii);
				}

				collected_combined_scores[bbb] = combined_scores;

			}

			vector<double> confidence[MATCHING_METHODS_COUNT];

			for (int bbb = 0; bbb < MATCHING_METHODS_COUNT; bbb++) {
				confidence[bbb].resize(collected_combined_scores[bbb].size());
				for (int iii = 0; iii < collected_combined_scores[bbb].size(); iii++) {
					double ratio = (iii == 0) ? localBlobMatchScores[bbb].at(collected_combined_scores[bbb].at(iii).second)/localBlobMatchScores[bbb].at(collected_combined_scores[bbb].at(1).second) : localBlobMatchScores[bbb].at(collected_combined_scores[bbb].at(iii).second)/localBlobMatchScores[bbb].at(collected_combined_scores[bbb].at(0).second);
					double conf = calculateConfidence(ratio, objI.viewRatioPoints[bbb]);
					confidence[bbb].at(collected_combined_scores[bbb].at(iii).second) = conf;
				}
			}

			// Combine the confidences to get one vector
			vector<double> pseudoConfidences;
			for (int iii = 0; iii < confidence[0].size(); iii++) pseudoConfidences.push_back(0.0);

			for (int xxx = 0; xxx < MATCHING_METHODS_COUNT; xxx++) {
				for (int iii = 0; iii < confidence[xxx].size(); iii++) {
					pseudoConfidences.at(iii) += confidence[xxx].at(iii);
				}
			}

			// Sort this vector to get the final indices
			vector<score_pair> combined_scores;
			for (int iii = 0; iii < confidence[0].size(); iii++) {
				score_pair sp(pseudoConfidences.at(iii), iii);
				combined_scores.push_back(sp);
			}
			std::sort(combined_scores.begin(), combined_scores.end());

			int correctIndexRank = -1;
			for (int iii = 0; iii < combined_scores.size(); iii++) {
				if (combined_scores.at(iii).second == trueViewIndex) {
					correctIndexRank = iii;
				}
			}

			// Rank prop for combined scores: closer to 1 is better because you are working with confidence/probability levels
			double rankProp = double(correctIndexRank)/double(combined_scores.size()-1);

			combined_rank_scores.push_back(rankProp);

			cv::Mat dMat = expandAndDisplay(viewMatchingMat, "viewMatchingMat", 1.0);
			cv::waitKey(1);

			int percentageComplete = percentageStep*int(floor((100.0/double(percentageStep))*double(aaa+1)/double(collectedBlobIdentities.size())));
			if (percentageComplete > lastPercentageReported) {
				std::printf("Pre-registration %d%% complete.\n", percentageComplete);
				lastPercentageReported = percentageComplete;
			}		
		}

		//cout << "dMat = " << dMat << endl;

		std::printf("View matching completed. (%d) known views tested.\n", actualViewTests);

		for (int aaa = 0; aaa < MATCHING_METHODS_COUNT; aaa++) viewRatiosFile[aaa].close();

		if (actualViewTests > 0) {
			
			int nObjectsWithValidViewTests = 0;
			for (int iii = 0; iii < viewMatchingCounts.size(); iii++) if (viewMatchingCounts.at(iii) > 0) nObjectsWithValidViewTests++;
			
			std::cout << "\nAverage accuracy of correct view for methods (" << nObjectsWithValidViewTests << "/" << viewMatchingCounts.size() << " objects tested) = " << endl;
			int precision = 2;


			for (int bbb = 0; bbb < MATCHING_METHODS_COUNT; bbb++) {
				double aver = 0.0;
				for (int iii = 0; iii < viewMatchingScores[bbb].size(); iii++) {
					if (viewMatchingCounts.at(iii) > 0) aver += (viewMatchingScores[bbb].at(iii) / (nObjectsWithValidViewTests * viewMatchingCounts.at(iii)));
				}
				cout << "[";
				cout << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << aver;
				cout << "]";
			}

			double totalScore = 0.0;
			for (int iii = 0; iii < combined_rank_scores.size(); iii++) {
				totalScore += combined_rank_scores.at(iii);
			}
			totalScore /= double(combined_rank_scores.size());
			cout << " combined = " << std::fixed << setw(precision+2) << std::setprecision(precision) << setfill('0') << totalScore << endl;

			cout << endl;
		}
	} else cout << "View matching can only be tested with preMatching flag set <true>." << endl;

	cout << endl;

	// Final summary of results
	objI.displayResults();
	objI.outputResults();

	std::printf("\n%s << Exiting... (press any key)\n", __FUNCTION__);
	cin.get();

#if defined(_WIN32) && defined(_DEBUG) && defined(TerminateProcess)
	TerminateProcess(GetCurrentProcess(), EXIT_SUCCESS);
#endif

	return 0;
}
