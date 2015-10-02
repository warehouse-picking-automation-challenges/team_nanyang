#ifndef OBJECT_RECOGNITION_TOOLS_H
#define OBJECT_RECOGNITION_TOOLS_H

using namespace std;

#include <iostream>
#include <iomanip>
#include <fstream>
#include <numeric>
#include <vector>
#include <algorithm>
#include <string>

#ifdef _IS_WINDOWS_
#include <tchar.h>
#include <windows.h>
#include <time.h> 
#include <io.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifndef sprintf_s
#define sprintf_s sprintf
#endif

#ifdef _OPENCV_VERSION_3_PLUS_
#define RETR_TREE cv::RETR_TREE
#define CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#else
#define RETR_TREE CV_RETR_TREE
#define CHAIN_APPROX_SIMPLE CV_CHAIN_APPROX_SIMPLE
#endif

#ifndef CHAIN_APPROX_SIMPLE
#define CHAIN_APPROX_SIMPLE CV_CHAIN_APPROX_SIMPLE
#endif

#define DEBUG_SEGMENT_SIZE cv::Size(200, 200)

#define SINGLE_OBJECT_FORMAT		0
#define MULTI_OBJECT_FORMAT			1

#define NEIGHBOUR_FRACTION 0.05

#define MIN_MATCHES				5
#define DEFAULT_LEVELS_PER_MM 5

#define DEFAULT_RETURNED_VIEWS 5

#define DEFAULT_EROSION_SIZE 3
#define DEFAULT_STRUCTURING_UNIT getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*DEFAULT_EROSION_SIZE + 1, 2*DEFAULT_EROSION_SIZE+1 ), cv::Point( DEFAULT_EROSION_SIZE, DEFAULT_EROSION_SIZE ) )

static int NP_DUMMY_1 = 0;
static int NP_DUMMY_2 = 0;

int float2int(float val);

void cleanMask(const cv::Mat& src, cv::Mat& dst);

bool readRotationFromTxtFile(ifstream& rotationsFile, vector<double>& rotation);
bool readRotationFromAlnFile(ifstream& rotationsFile, vector<double>& rotation);

string get_address(string address_file, string default_address);

int countBlobsInMask(const cv::Mat& src);

cv::Mat appendImage(const cv::Mat& im1, const cv::Mat& im2);

void addToDebugImage(const cv::Mat& src, cv::Mat& dst, int row = 0, int col = 0);

bool is_contained(int val, vector<int>& arr);

void writeMatrixToCSV(const cv::Mat& src, string filename, vector<string> objectNames = vector<string>());
void convertProbabilityMat(const cv::Mat& src, cv::Mat& dst, bool setAbsentSelfTestsToZero = false);

double calculateProbabilityAccuracy(int elements, double *limits, int tallies[][2]);

void generateIdentityConfigurations(vector<cv::Mat>& out, int blob_count, int class_count = -1);
void generatePermutations(int n, vector<int>& A, vector<vector<int> >& super_vec);
void makePermutationsUnique(vector<vector<int> >& super_vec);

cv::Mat generateMask(const cv::Mat& src, int blobs = 1);

bool hasExtension(string& src, string& ext);
void showViewMatchingResult(const vector<double>& scores, int views_per_level);
double viewCompatibilityScore(const vector<double>& scores, int correct_index, int views_per_level);

double calculateViewError(int level_index_1, int frame_index_1, int level_index_2, int frame_index_2, int level_count, int frame_count);

bool getContents(string src, vector<string>& contents);
void filterContents(vector<string>& contents, const vector<int>& elevation_indices = vector<int>(), bool is_mask = false);

void makeDirectory(string dir_name);

double calculateConfidence(const double ratio, const vector<double>& interpolationPoints);
void generateConfusionMatrix(const cv::Mat& success, const cv::Mat& failure, cv::Mat& dst);
void runSimulatedIdentityTests(const vector<vector<vector<double> > >& scores, const vector<double>& confidenceFormula, cv::Mat& success, cv::Mat& failure, vector<vector<double> >& resultsRatios, int& tripleTestCount, int& tripleTestSuccess);
cv::Mat expandAndDisplay(const cv::Mat& src, string windowName = "result", double max_val = 0.0);
cv::Mat generateSpecificMask(const cv::Mat& src, int index = 0);
cv::Mat generateCombinedMask(const cv::Mat& src);

void displayKeyPoints(const cv::Mat& image, const vector<cv::KeyPoint>& fPoints, const vector<cv::KeyPoint>& ePoints, cv::Mat& outImg, int thickness = 1, bool pointsOnly = false);
void determineObjectEdgePixels(const cv::Mat& mask, vector<cv::Point>& pixelLocs);
bool doesFeatureIntersect(const vector<cv::Point>& pixelLocs, const cv::KeyPoint& kp);
void createMatchingMatrix(cv::Mat& matchingMatrix, const cv::Mat& desc1, const cv::Mat& desc2);
void twoWayPriorityMatching(cv::Mat& matchingMatrix, vector<vector<cv::DMatch> >& bestMatches);
void sortMatches(vector<vector<cv::DMatch> >& matches1to2);
void filterMatchesByClass(vector<vector<cv::DMatch> >& matches1to2, const std::vector<int> im1foreground, const std::vector<int> im1edge, const std::vector<int> im2foreground, const std::vector<int> im2edge);
double calculateScoreFromDMatchVector(const vector<vector<cv::DMatch> >& matches1to2);
bool readXfTransformation(const std::string& src, cv::Mat& dst);

double getPercentileValue(const vector<double>& vals, double percentile);

void findBestScores(const vector<vector<double> >& matchingScores, vector<double>& scores, vector<int>& id1, vector<int>& id2, int num, int views_per_level);

void generateMatchScoreMatrix(vector<vector<double> > scores, cv::Mat& result, vector<int>& match_obj, vector<int>& match_img, int views_per_level, int obj_id = -1, int img_id = -1);
double colourHistogramMatchScore(cv::Mat& test_hist, cv::Mat& lib_hist);
double colourHistMatch(const cv::Mat& test_hist, const cv::Mat& lib_hist);
double dimensionsMatch(double width_1, double height_1, double width_2, double height_2);
double featuresMatch(const cv::Mat& foreground_descs_1, const cv::Mat& edge_descs_1, const cv::Mat& foreground_descs_2, const cv::Mat& edge_descs_2);
void normalizeHist(cv::Mat& src, cv::Mat& dst);
void smoothMatchingScores(vector<vector<double> >& scores, int views_per_level);
void findNeighbours(const int& center, vector<int>& indices, int views_per_level);
void drawHistogram(const cv::Mat& src, cv::Mat& dst);

double contourRatio(const vector<cv::Point>& contour);

bool duplicateString(const vector<string>& string_vec);

void histogramNormalize(const cv::Mat& src, cv::Mat& dst, const cv::Mat &mask, float percent_lo = 1.0, float percent_hi = 1.0);

int getNPVal(string input, int& num_1 = NP_DUMMY_1, int& num_2 = NP_DUMMY_2);

bool readImageDir(string directory, string& save_directory, std::vector<string>& objFolders, std::vector<std::vector<string> >& imgNames, std::vector<std::vector<string> >& maskNames, const vector<int>& elevations = vector<int>(), const vector<int>& indices = vector<int>());

void saveImage(char *dir, int obj_idx, int img_idx, std::vector<string>& objFolders, std::vector<std::vector<string> >& imgNames, cv::Mat& im);

cv::Mat cropImage(const cv::Mat& src, const cv::Mat& mask = cv::Mat(), int buffer = 5, bool remask = true);
cv::Mat cropImageFixed(const cv::Mat& src, const cv::Mat& mask = cv::Mat(), int width = 200, bool remask = true);
cv::Mat buffImage(const cv::Mat& src, cv::Size sz = DEBUG_SEGMENT_SIZE);
cv::Mat extractObject(const cv::Mat& src, const cv::Mat& mask);

void findExtents(const cv::Mat& mask, int *extents = NULL);

void padNames(vector<string>& names, bool is_mask = false);
void unpadNames(vector<string>& names);

void fillHoles(const cv::Mat& src, cv::Mat& dst, int fillType);

bool compareByResponse(cv::KeyPoint a, cv::KeyPoint b);


#endif // OBJECT_RECOGNITION_TOOLS_H
