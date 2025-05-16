# Visualization of point cloud
This document shows how to visualize point clouds by using PCL.


## Preparation
Open [pcl_tool.cc](../tool/pcl_tool.cc) 

### Enable viewer
```cpp
#define ENABLE_VIEWER;  //Remove code comments
```

##### Visualization of online point cloud 
```cpp
// assign param
param.input_param.source_type = DATA_FROM_LIDAR;  //Set the data source to real-time data
param.input_param.ptc_port = 9347;  //TCP protocol port
param.input_param.udp_port = 2368;  //UDP protocol port
param.input_param.host_ip_address = "192.168.1.100";  //Destination IP Address
param.input_param.firetimes_path = {"Your firetime file path"};   // Optional：Laser firing sequence (Firetimes file path)
```
##### Visualization of PCAP point cloud data
```cpp
// assign param
param.input_param.source_type = DATA_FROM_PCAP; //Set the data source to PCAP data
param.input_param.pcap_path = {"Your pcap file path"};  // PCAP file path
param.input_param.correction_file_path = {"Your correction file path"};   //Calibration file path (Angle Correction file path)
param.input_param.firetimes_path = {"Your firetime file path"}; // Optional：Laser firing sequence (Firetimes file path)
```


## Steps
### 1 Compile
Navigate to the HesaiLidar_SDK_2.0 directory, open a Terminal window, and run the following commands.
```cpp
cd HesaiLidar_SDK_2.0/tool
mkdir build
cd build
cmake ..
make
```

### 2 Run
Once compiled successfully, go to the build folder, execute the generated pcl_tool executable, and the system will display a visualization window.
```cpp
./pcl_tool
```

