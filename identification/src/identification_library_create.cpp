#include "identification.h"

// SETTINGS
#define DEBUG_MODE true
static const int np_angles[] = {-1}; // [-1 for all] // Berkeley Data: 1-5 valid
static const int object_id[] = {-1}; // [-1 for all] // Berkeley Data: 1-25 valid

int main(int argc, char *argv[]) {

	vector<int> objIdents (object_id, object_id + sizeof(object_id) / sizeof(object_id[0]) );
	vector<int> npAngles (np_angles, np_angles + sizeof(np_angles) / sizeof(np_angles[0]) );

	string output_library_address = string(_SOURCE_DIRECTORY_) + "/yml_library/output";
    string raw_data_address = get_address("raw_data_address.txt", "image_library/image_library");
	
	objectIdentifier objI;
	objI.restrictData(objIdents, npAngles);

	objI.setDevelopmentDirectory(string(_SOURCE_DIRECTORY_) + "/output");
	
	// Loops through and creates all the YML files
	cout << "Generating library YML files..." << endl;
	objI.generateLibrary(raw_data_address, output_library_address, DEBUG_MODE);

	std::printf("%s << Exiting... (press any key)\n", __FUNCTION__);
	cin.get();

#if defined(_WIN32) && defined(_DEBUG) && defined(TerminateProcess)
	TerminateProcess(GetCurrentProcess(), EXIT_SUCCESS);
#endif

	return 0;
}
