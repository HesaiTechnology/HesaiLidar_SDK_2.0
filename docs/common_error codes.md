# Common_error codes
## 1 Introduction
Here are some common warning codes that you may encounter.
## 2 Troubleshooting
1. `Failed to obtain correction file from lidar!`
   This indicates that the program attempted to retrieve the angle correction data directly from the lidar but failed. After that, it still tries to load the correction file from a local path specified in the configuration `param.input_param.correction_file_path`. If a valid local file is found, the program can still proceed.

2. `Packet with invaild delimiter`
   This indicates that the program either read a data packet with an incorrect format or failed to read an expected packet. Please check the data path and verify the data's integrity.

3. `Firetimes laserId is invalid`
   This indicates that the program read an incorrect firetime correction file.

4. `Open firetime file failed`
   This indicates that the program failed to read the firetime file, but it will not prevent the program from running.