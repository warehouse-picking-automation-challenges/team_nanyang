#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#ifdef _IS_LINUX_
#include "ros/ros.h"
#include "identification/SRV_Identification.h"
#endif

using namespace std;

#include "tools.h"

#define MIN_PIXELS_FOR_VALID_MASK	50
#define DEFAULT_DESIRED_FEATURES	300
#define FEATURES_PER_TYPE			50
#define VIEWS_PER_LEVEL				120
#define DEFAULT_BIN_NUM				16
#define SAMPLES_FOR_IDENTITY_TEST	300

// Testing methods
#define SINGLE_OBJECT_PER_IMAGE		0
#define IDENTIFICATION_MATRIX		1
#define PRE_REGISTRATION			2
#define MULTI_OBJECT_PER_IMAGE		4

// Matching Methods
#define MATCHING_METHODS_COUNT		4
#define MM_HISTOGRAM_COMPARISON		0 // 0.9931
#define MM_FEATURES_COMPARISON		1 // 0.9278
#define MM_DIMENSIONS_COMPARISON	2 // 0.9192
#define MM_CONTOUR_COMPARISON		3 // 0.7833

#define FEATURES_SCALE_FACTOR		1

static double probability_thresholds[] = {0.99, 0.99, 0.99, 0.99}; // latest results (0.9954)

static int active_matching_methods[] = {-1}; // set as -1 to use all methods 

typedef std::pair<double,int> score_pair;
inline bool comparator ( const score_pair& l, const score_pair& r) { return l.first < r.first; }
static cv::Mat _emptyMat;

/// \brief For storing results of an individual test
struct testResult {
	int true_id;
	vector<int> false_ids;
	double offlineConfidence, onlineConfidence;
	cv::Mat priorProbs;
	bool result;
	vector<vector<int> > matchedIndices;

	inline testResult() : true_id(-1), offlineConfidence(-1.0), onlineConfidence(-1.0) { };
};

/// \brief For storing data associated with an individual object
class blob {
public:
	string objectName, viewName;
	double width, height;
	cv::Mat foregroundDescs, edgeDescs, colourHist;
	cv::Mat croppedBlob, featuresIm;
	vector<cv::Point> contour;
	std::vector<cv::KeyPoint> foregroundFeatures, edgeFeatures;
	vector<double> rotation;
};

double calculateMatchingScore(const blob& blob_1, const blob& blob_2, int method = MM_HISTOGRAM_COMPARISON);

/// \brief For storing data associated with an individual image
class imset {
	friend class imageDirectory;
	friend class objectLibrary;
	friend class objectIdentifier;

public:
	void assign(const cv::Mat& img, const cv::Mat& mask, const cv::Mat& depth = cv::Mat());
	void describe(bool debug = false, std::string dir = std::string());
	cv::Mat getDebugIm(int blob_index = 0, cv::Scalar color = cv::Scalar(-1.0, -1.0, -1.0));
	void release();
	void showImage();
	inline void saveFeatureImage(string filename, int idx = 0) { if (blobs[idx].featuresIm.rows > 0) cv::imwrite(filename, blobs[idx].featuresIm); }
	
	void generateFeatureImages(bool use_mask = false);
	void showFeatureImages(int idx = -1, string custom_name = "");
	void classifyFeatures();
	void detectFeatures(int desiredFeatures = DEFAULT_DESIRED_FEATURES);
	void describeFeatures();
	void setDescName(int idx = 0);
	void extractCurrentContours();
	void generateColourHistograms();
	void measureDimensions();
	inline string getDescName(int idx) { return (blobs[idx].viewName + ".yml"); }

	vector<blob> blobs;

protected:
	string rgbName, maskName, depthName;
	
	cv::Mat transform;
	cv::Mat inputImage, inputMask, inputDepth;
	cv::Mat grayIm, grayColIm, expandedIm, expandedMask;
	vector<cv::KeyPoint> detectedPoints;
	cv::Mat descs;
	
};

int countDifferentLevels(vector<imset> *image_sets);

inline size_t countViews(vector<vector<imset> > *objVV) { 
	size_t retVal = 0;
	for (size_t iii = 0; iii < objVV->size(); iii++) retVal += objVV->at(iii).size();
	return retVal;
}

/// \brief For images stored in the standard directory structure
class imageDirectory {
	friend class objectLibrary;
	friend class objectIdentifier;

public:
	inline imageDirectory() : directoryFormat(SINGLE_OBJECT_FORMAT), images(NULL) { }
	inline void setImageDirectory(string dir) { image_directory = dir; }
	void loadImageLocations();
	bool traverseDirectory();
	inline size_t getObjectCategoryCount() { return objectFolders.size(); }
	inline size_t getImageCount(int idx) { return images->at(idx).size(); } 
	size_t getTotalImageCount();
	size_t getTotalBlobCount();
	void loadImage(int obj_idx, int img_idx);
	void retrieveImage(int obj_idx, int img_idx, cv::Mat& img, cv::Mat& mask, cv::Mat& depth = _emptyMat);
	void saveImage(char *dir, int obj_idx, int img_idx, cv::Mat& im, string name);
	void getIdentities(int obj_idx, int img_idx, vector<string>& object_names, vector<string>& view_names);

	void limitNPAngles(const vector<int>& npVals);
	inline void limitObjects(const vector<int>& ids) { acceptableObjectIndices = ids; }
	int findObjectIndex(string object_name);
	int findViewIndex(string object_name, string image_name);

	std::vector<std::vector<imset> > *images;
	std::vector<string> objectFolders;

protected:
	
	int directoryFormat;
	bool verboseMode;
	string image_directory;
	vector<int> allowableElevations, acceptableObjectIndices;
	std::vector<std::vector<cv::Mat> > transformations;
};

/// \brief For generating and storing an identification library
class objectLibrary : public imageDirectory {
	friend class objectIdentifier;

public:
	inline void setYMLDirectory(string dir) { yml_directory = dir; }
	bool loadExisting(string lib_address, string img_address = string());
	
	bool fullyMatch(const blob& input_blob, vector<string> potential_objects, bool debug = false);

	void loadTransformation(int obj_idx, int img_idx);
	bool estimateBestMatchFromName(const blob& input_blob, int& est_obj_id, int& est_img_id);
	void calculateIdentificationScores(const vector<vector<double> >& matchingScores, vector<double>& identificationScores);
	void generate(bool debug = false);
	void addDummyObjects(vector<string>& object_names, int num);
	bool prepareLibraryDirectories();
	void saveFeatureLibraryComponent(int obj_idx, int img_idx);
	void initializeOutputFile();
	inline void closeOutputFile() { libraryGenSummary.close(); }

	void retrieveSpecificMatchingResults(vector<double> *blobMatchScoreVector, int object_idx);
	void retrieveMatchingResults(vector<vector<vector<double> > > *blobMatchScoreVector);
	void clearPreviousResults();

protected:

	vector<vector<double> > blobMatchScores[MATCHING_METHODS_COUNT];

	vector<vector<double> > combinedIdTestScores;
	vector<vector<double> > identificationTestScores[MATCHING_METHODS_COUNT]; // for a single blob, to store all distances e.g. for histogram, contour and features
	vector<vector<double> > registrationTestScores[MATCHING_METHODS_COUNT];
	string yml_directory;
	ofstream libraryGenSummary;

};

// Interface for performing real-time identification
class objectIdentifier {
public:

#ifdef _IS_LINUX_
    objectIdentifier(ros::NodeHandle& nh, string library_address, string params_directory);
    ros::ServiceServer service;
    void advertise_service();
    bool identification(identification::SRV_Identification::Request  &req, identification::SRV_Identification::Response &res);
#endif

	inline objectIdentifier() { init(); }

	// Real-time Use
	inline bool loadLibrary(string dir, string img_dir = string()) { return lib.loadExisting(dir, img_dir); }
	void supplyLatestImage(const cv::Mat& rgbImage, const cv::Mat& maskImage, const cv::Mat& depthImage = cv::Mat());
    void identifyObject(const vector<std::string>& objectNames, int index, int numViews, int& blob_index, double& confidence, vector<vector<double> >& viewRotations, vector<std::string>& versionNames, double& preConfidence, bool debug = false, bool avoid_best = false, bool evaluation_mode = false);

	// Internal
	void init();
	int findMatchingIdentity(const vector<std::string>& objectNames, double& confidence, int searchObjectIndex, bool debug = false, int *blob_indices = NULL, double *blob_confidences = NULL);
    void findMatchingViews(int blobIndex, int bestLibraryIndex, vector<string>& bestViewLabels, vector<vector<double> >& bestRotations, double& preconf, int num_views = DEFAULT_RETURNED_VIEWS, bool debug = false, bool avoid_best = false);

	// Development
	void updateIdentificationMatrices(const vector<string>& objects, const vector<int>& true_indices, int *result_index, double *confidence, bool debug = false);
	cv::Mat generateDisplayImage(cv::Rect roiRect = cv::Rect());
	void displayMatch(int guessed_index);
	void setDevelopmentDirectory(string dir) { resultsOutputDir = dir; }
	void restrictData(const vector<int>& obj_ids, const vector<int>& img_ids);
	void generateLibrary(string src, string dst, bool debug = false);
	void identificationEvaluation();
	void setOutputDir(string dir) { resultsOutputDir = dir; }
	void outputResultSummary(string realObject, string realName);
	bool outputIdentificationResultsSummary(const vector<string>& objects, int true_index, int result_index);
	inline void addDummyObjects(vector<string>& object_names, int num = 3) { lib.addDummyObjects(object_names, num); };
	void updateIdentificationMatrices(const vector<double> *identificationScores, const int& object_id);
	void outputResults();
	void displayResults();
	void clearPreviousResults();
	bool outputConfidenceData(const vector<std::string>& objectNames, vector<vector<vector<double> > > *blobMatchScoreVector);

	void supplyPrematchingResults(vector<vector<vector<double> > > *blobMatchScoreVector);
	bool processMatchingResults(const vector<std::string>& objectNames, const vector<std::string>& viewNames, vector<vector<vector<double> > > *blobMatchScoreVector);
	// void processViewResults();

	void displayCurrentMatchingResults();
	void saveCurrentBlobMatchScores();
	void loadConfidencePoints(std::string params_directory = "");
	void initializeBlobMatchScoreStorage();
	void generateFailureImage(const vector<string>& objects, bool is_correct = false);

	objectLibrary lib;

	vector<double> viewRatioPoints[MATCHING_METHODS_COUNT];

protected:

	string confidencePointsFiles[MATCHING_METHODS_COUNT], viewRatioPointsFiles[MATCHING_METHODS_COUNT];

	testResult currentTest;
	vector<testResult> testResults;

	std::vector<int> amm;

	imset testImage;
	int bestMatchingIndex;
	int counter;
	bool verboseMode;
	
	string resultsOutputDir;
	string searchName;
	int currentBlob;
    vector<string> currentViewLabels, currentVersionNames;
    vector<vector<double> > currentViewRotations;
	double currentConfidence, currentPreConfidence;
	cv::Mat idSuccess[MATCHING_METHODS_COUNT+1], idFailure[MATCHING_METHODS_COUNT+1], idPriorProb[MATCHING_METHODS_COUNT+1];
	cv::Mat *idSuccess3[MATCHING_METHODS_COUNT+1], *idFailure3[MATCHING_METHODS_COUNT+1], *idPriorProb3[MATCHING_METHODS_COUNT+1];
	vector<double> confidenceRecord[2], confidenceRecord3[2];
	vector<vector<double> > resultsRatios;
	vector<vector<vector<double> > > blobMatchScores[MATCHING_METHODS_COUNT];
	vector<double> identityConfidencePoints[MATCHING_METHODS_COUNT];
	cv::Mat testCountMatrix[MATCHING_METHODS_COUNT], identificationMatchingScoreMatrix[MATCHING_METHODS_COUNT];

	ofstream identityConfidenceData[MATCHING_METHODS_COUNT];
    string confidence_file[MATCHING_METHODS_COUNT];
	
};

#endif // LIBRARY_GENERATION_H
