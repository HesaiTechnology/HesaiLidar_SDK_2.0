# HesaiLidar_SDK_2.0
## About the project
This repository includes the software development kit for Hesai LiDAR sensor manufactured by Hesai Technology

## Support Lidar type
- Pandar
- AT128/AT128P
- QT
- FT120
- XT16/XT32
- ET25/ET30
- OT
- ATX
- JT16
- JT128、JT256 (need define JT128_256)

## Environment and Dependencies

**System environment requirement:Linux**
```
Recommanded
-Ubuntu 16.04
-Ubuntu 18.04
-Ubuntu 20.04
-Ubuntu 22.04
-Windows 10
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

## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git

Note: Window is not recommended to use the compressed package, there will be symbol problems lead to compilation errors
```

**Note when parsing JT128/JT256**
```
Need to add macro definition JT128_256, can add statement in CMakeLists.txt --- ‘add_definitions(-DJT128_256)’
```

## Ubuntu Build
```
1.$ cd HesaiLidar_SDK_2.0
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```

## window Build
```
Environmental preparations:
	- Visual Studio2022	 https://visualstudio.microsoft.com/zh-hans/downloads/
	- cmake-gui  		 https://cmake.org/download/
	- Git 				 https://git-scm.com/
	- OpenSSL v1.1.1	 https://slproweb.com/products/Win32OpenSSL.html
Note: Modify 'set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")' to the actual path

Compile Environment Configuration:
	1. Open CMake-GUI, select the source directory (`HesaiLidar_SDK_2.0`) and output directory (`HesaiLidar_SDK_2.0/build`)
	2. Click `Configure`
  	3. Click `Generate`
	4. If it prints “Configuring done” and “Generating done” then it is OK, otherwise check for errors.
	5. Click on `Open Project`
	6. Right click `ALL BUILD` and click `Generate`
	7. The corresponding executable file can be found in the `Debug/Release` folder under `Build`
```

## Run a sample

Set the parameters in param in main.cc or main.cu
```
// Reciving data from pcap file
```
	param.input_param.source_type = DATA_FROM_PCAP;
	param.input_param.pcap_path = "path/to/pcap";
	param.input_param.correction_file_path = "/path/to/correction.csv";
	param.input_param.firetimes_path = "path/to/firetimes.csv";
	param.decoder_param.distance_correction_lidar_flag = false;   // Set to true when distance correction needs to be turned on
```
// Reciving data from connected Lidar
```
	param.input_param.source_type = DATA_FROM_LIDAR;
	param.input_param.device_ip_address = '192.168.1.201'; // 192.168.1.201 is the lidar ip address
	param.input_param.ptc_port = 9347; // 9347 is the lidar ptc port
	param.input_param.udp_port = 2368; // 2368 is the lidar udp port
	param.input_param.host_ip_address = "192.168.1.100"; // 192.168.1.100 is the pc ip address
	param.input_param.multicast_ip_address = "239.0.0.1"; // 239.0.0.1 is the lidar multcast ip address, set this parameter to "" when lidar do not support muticast
	param.input_param.correction_file_path = "/path/to/correction.csv";
	param.decoder_param.distance_correction_lidar_flag = false;   // Set to true when distance correction needs to be turned on
```
// Reciving data from connected Lidar (Serial data)
```
	param.input_param.source_type = DATA_FROM_SERIAL;
	param.input_param.rs485_com = "Your serial port name for receiving point cloud"; 
  	param.input_param.rs232_com = "Your serial port name for sending cmd"; 
	param.input_param.correction_file_path = "/path/to/correction.csv";
	param.decoder_param.distance_correction_lidar_flag = false;   // Set to true when distance correction needs to be turned on
```

$ make 
// run a cpu sample
$ ./sample
// run a gpu sample
$ ./sample_gpu
```

## Functional Parameter Description
```
DecoderParam :
	1. pcap_play_synchronization: When parsing pcap, it is delayed according to the point cloud time to simulate the publish speed when parsing in real time.
	2. frame_start_azimuth: Split-frame position of the 360-degree rotating lidar (in degrees [0-360)).
	3. enable_packet_loss_tool: Packet loss statistics, based on sequence number.
	4. enable_packet_timeloss_tool: Packet loss statistics, based on whether the timestamp is backed up or not, only once.
	5. packet_timeloss_tool_continue: Whether packet loss statistics are continuously performed (based on timestamps).
	6. use_timestamp_type: Use timestamp type (point cloud carry or local time).
	7. fov_start and fov_end: Allowable light emission angle, outside the angle is set to 0.
	8. distance_correction_lidar_flag: Control of optical center corrections for mechanical lidar.
	9. xt_spot_correction: Controlling point cloud S-stratification corrections for the XT16/32 lidar.
	10. Setting the buffer size of the system udpsocket.
InputParam:
	1. ptc_mode: ptc mode, some lidar support ssl encrypted communication.
	2. source_type: udp data sources.
	3. device_ip_address: Lidar ip, used for ptc connections, also used to filter udp data if filtering is enabled
	4. multicast_ip_address: Enabled when lidar point cloud udp is multicast.
	5. device_udp_src_port: On at >=1024, filter point cloud data for specified ports and ip(device_ip_address).
	6. device_fault_port: On at >=1024, filter fault message data for specified ports and ip(device_ip_address).
	7. udp_port: point cloud udp port.
	8. ptc_port: lidar ptc port, used for ptc connections.
	9. rs485_com: Point cloud port for serial reception (only for JT16).
	10. rs232_com: Command send port for serial port reception (only for JT16).
	11. point_cloud_baudrate: Point cloud data baud rate (only for JT16).
	12. rs485_baudrate: OTA command baud rate (only for JT16).
	13. rs232_baudrate: Normal command baud rate (only for JT16).
	14. correction_save_path: Serial port to get the storage path of the angle calibration file (only for JT16).
	15. pcap_path: Local path to pcap when pcap parses.
	16. correction_file_path: Local path to correction file.
	17. firetimes_path: Local path to firetimes file.
	18. certFile、privateKeyFile、caFile: The storage path for certificates and keys when communicating with ptcs.
	19. standby_mode: Initialization sets the lidar mode to * (on if not -1).
	20. speed: Initialization sets the lidar speed to * (on if not -1).
DriverParam: 
	1. log_level: Log level to be output.
	2. log_Target: Log output location, print and file.
	3. log_path: File location for log output.
```
