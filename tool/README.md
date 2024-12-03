# HesaiLidar_SDK_2.0
## About the project
This repository includes the software development kit tools for Hesai LiDAR sensor manufactured by Hesai Technology

## Support tools
- packet_loss_tool
- pcl_tool
- ptc_tool
- las_tool

## Environment and Dependencies

**System environment requirement:Linux**
```
Recommanded
-Ubuntu 16.04
-Ubuntu 18.04
-Ubuntu 20.04
-Ubuntu 22.04
```

**Compiler vresion requirement**
```
Cmake version requirement:Cmake 3.8.0 or above
G++ version requirement:G++ 7.5 or above
```
**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev
```
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev
```
**Library Dependencies: LASlib**  
We use the following open source repositories to compile the LASlib library:  
https://github.com/LAStools/LAStools  
Install according to the library tutorial  
Note:  
If encountering compilation errors: undefined reference to `dlopen', Please add the following line in the LASlib/src/CMakeLists.txt file:
```
target_link_libraries(LASlib ${CMAKE_DL_LIBS})
```
After compilation, please install:
```
sudo make install
```
If prompted for missing laszip_common h:
```
cd LASzip/include/laszip
sudo cp laszip_api_version.h /usr/local/include/LASlib
sudo cp laszip_common.h /usr/local/include/LASlib
```
## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git
```

## Build
```
1.$ cd HesaiLidar_SDK_2.0
2.$ cd tool
3.$ mkdir build
4.$ cd build
5.$ cmake ..
6.$ make
```
## Tool Description
- packet_loss_tool: This is a tool used for packet loss detection and packet timestamp bounce detection
- pcl_tool: This is a tool that uses the PCL tool to generate point cloud data in PCD, ply, and bin formats
- ptc_tool: This is a tool used to control lidar using PTC connection
- las_tool: This is a tool that uses the LASlib library to generate point cloud data in las and laz formats


## Run a sample
Firstly, it is necessary to configure the radar connection information
Taking pcl_tool as an example

Set the parameters in param in pcl_tool.cc or pcl_tool.cu
```
// Reciving data from pcap file
```
```
	param.input_param.source_type = DATA_FROM_PCAP;
    param.decoder_param.enable_packet_loss_tool = false;
    param.decoder_param.enable_packet_timeloss_tool = false;
    param.decoder_param.packet_timeloss_tool_continue = false;
	param.input_param.pcap_path = "path/to/pcap";
	param.input_param.correction_file_path = "/path/to/correction.csv";
	param.input_param.firetimes_path = "path/to/firetimes.csv";
	param.decoder_param.distance_correction_lidar_flag = false;   // Set to true when distance correction needs to be turned on
```
```
// Reciving data from connected Lidar
```
```
	param.input_param.source_type = DATA_FROM_LIDAR;
    param.decoder_param.enable_packet_loss_tool = false;
    param.decoder_param.enable_packet_timeloss_tool = false;
    param.decoder_param.packet_timeloss_tool_continue = false;
	param.input_param.device_ip_address = '192.168.1.201'; // 192.168.1.201 is the lidar ip address
	param.input_param.ptc_port = 9347; // 9347 is the lidar ptc port
	param.input_param.udp_port = 2368; // 2368 is the lidar udp port
	param.input_param.host_ip_address = "192.168.1.100"; // 192.168.1.100 is the pc ip address
	param.input_param.multicast_ip_address = "239.0.0.1"; // 239.0.0.1 is the lidar multcast ip address, set this parameter to "" when lidar do not support muticast
	param.decoder_param.distance_correction_lidar_flag = false;   // Set to true when distance correction needs to be turned on
```
Then, for each tool, you need modify some setting
### pcl_tool:  
Configure macro variables as needed  
```
    #define SAVE_PCD_FILE_ASCII //Configure PCL tool to generate pcd files
    #define SAVE_PCD_FILE_BIN //Configure PCL tool to generate bin files
    #define SAVE_PCD_FILE_BIN_COMPRESSED //Configure PCL tool to generate compressed bin files
    #define SAVE_PLY_FILE //Configure PCL tool to generate ply files
    #define ENABLE_VIEWER //Enable PCL Viwer
```
### ptc_tool:  
Set variables as needed  
```
    std::string destination_ip = "192.168.1.100";// set destination ip 
    uint16_t udp_port = 2368;// set destination  port
    uint16_t gps_udp_port = 10110;// set gpu port
    uint8_t return_mode = 1; // set return_mode
    uint8_t enable_sync_angle = 1; // set syncangle enable
    uint16_t sync_angle = 20; // set syncangle
    uint32_t standby_mode = 0; // Set standby_mode
    uint32_t speed = 1200; // set spin_speed
```
### las_tool:  
Configure macro variables as needed  
```
    #define SAVE_LAS_FILE //Configure LASlib library to generate las files
    #define SAVE_LAZ_FILE //Configure LASlib library to generate laz files
    //Note: If both of these are configured simultaneously, only the laz file will be generated in the end
```
### packet_loss_tool  
Set configuration as needed  
```
    param.decoder_param.enable_packet_loss_tool = false; //Enable packet loss detection tool
    param.decoder_param.enable_packet_timeloss_tool = false; //Enable timestamp bounce detection tool
    param.decoder_param.packet_timeloss_tool_continue = false; /* Set whether to perform persistent timestamp bounce detection. If not set, timestamp detection will be turned off when the first timestamp bounce is detected */
```
Afterwards, perform compilation to use the tool  
```
    $ make 
    // run a pcl_tool
    $ ./pcl_tool
    // run a ptc_tool
    $ ./ptc_tool
    // run a las_tool
    $ ./las_tool
    // run a packet_loss_tool
    $ ./ptc_toolpacket_loss_tool
```
