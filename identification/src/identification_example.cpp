#include "identification.h"

int main(int argc, char *argv[])
{
	return 0; // This program exists as a guide for using the <objectIndentifier> interface, and does not run properly on its own.

// ======================
// INITIALIZATION
// ======================

	// Set the address where the library YML files are stored
	string library_address = string(_SOURCE_DIRECTORY_) + "/yml_library/yml_library_sample";
	
	// Create the <objectIdentifier>
	objectIdentifier objI;

	// Load the library into the identifier
	objI.loadLibrary(library_address);

// ======================
// CALLBACK
// ======================

	// The following code will be called each time the identifier needs to be used:

	// Colour image and mask image from Image Capture module
	cv::Mat rgbImage, maskImage;

	// Possible identities from Task Planning module
	vector<string> trueIdentities;
	
	// Which element of the "trueIdentities" vector is the object that we want to find and pick up (also from Task Planning module)
	int targetIdentity;

	// How many (top N) views Registration module wants
	int returnedViews = DEFAULT_RETURNED_VIEWS;
	
	// Prepare variables for storing identification results
    int blob_index;                         // Which of the blobs in the mask image matches the desired identity (to be passed to Registration module)
    double confidence;                      // Estimated probability that estimate of which blob is the desired identity is correct (to be passed to Task Planning module)
    vector<vector<double> > viewRotations;	// Vector of rotation matrices (stored as 1 x 9 vectors) for the relative rotation between the library object and the specific matched view. Designed to help the Registration module.
    vector<string> versionNames;			// Vector of names of which version of the object the corresponding rotation applies to. Designed to help the Registration and Gripping module deal with objects that have different versions e.g. the glue.
	double preConfidence;                   // Estimated probability that the estimate of the best matching image is good enough to guide accurate registration (to be passed to the Registration module)

	// Perform the identification routine
    objI.identifyObject(trueIdentities, targetIdentity, returnedViews, blob_index, confidence, viewRotations, versionNames, preConfidence);

	// Optional: Displays the result to screen, for debugging/demo purposes
	cv::Mat resultImage = objI.generateDisplayImage();

	return 0;
}
