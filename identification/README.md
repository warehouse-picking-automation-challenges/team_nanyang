Team NTU Amazon Picking Challenge 2015 - identification module
==============================================================

Changing/setting the YML (identification) library
-------------------------------------------------
1. Download the latest library from Google Drive @ NTU-APC > Perception > Identification > Data > yml_library.zip
2. Copy and unpack it in the <apc\identification\yml_library> directory.
3. Add a file <yml_data_address.txt> to the <apc\identification\params> directory, and in this file, insert the full path to the unpacked YML directory.

Changing/setting the raw image library used to generate and debug the YML library
---------------------------------------------------------------------------------
1. Acquire the latest image library from someone in the Perception/Identification team. Originally it was based on the Berkeley data but this may have changed.
1b. Alternatively download the latest sample image library from Google Drive @ NTU-APC > Perception > Identification > Data > image_data_sample.zip
2. Copy and unpack it in the <apc\identification\image_library> directory.
3. Add a file <raw_data_address.txt> to the <apc\identification\params> directory, and in this file, insert the full path to the unpacked image directory.

Changing/setting the image test data
------------------------------------
1. Download the latest library from Google Drive @ NTU-APC > Perception > Identification > Data > test_data.zip
2. Copy and unpack it in the <apc\identification\test_data> directory.
3. Add a file <test_data_address.txt> to the <apc\identification\params> directory, and in this file, insert the full path to the unpacked test directory.
