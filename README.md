# HesaiLidar_SDK_2.0
## About the project
This repository includes the software development kit for Hesai LiDAR sensor manufactured by Hesai Technology

## Support Lidar type
- Pandar
- AT128/AT128P
- QT
- FT120
- XT16/XT32
- ET25
- OT

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
**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev + libboost-dev**
```
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev libboost-dev
```

## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git
```

## Build
```
1.$ cd HesaiLidar_SDK_2.0
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
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
```
// Reciving data from connected Lidar
```
	param.input_param.source_type = DATA_FROM_LIDAR;
	param.input_param.device_ip_address = '192.168.1.201'; // 192.168.1.201 is the lidar ip address
	param.input_param.ptc_port = 9347; // 9347 is the lidar ptc port
	param.input_param.udp_port = 2368; // 2368 is the lidar udp port
	param.input_param.host_ip_address = "192.168.1.100"; // 192.168.1.100 is the pc ip address
	param.input_param.multicast_ip_address = "239.0.0.1"; // 239.0.0.1 is the lidar multcast ip address, set this parameter to "" when lidar do not support muticast
```

$ make 
// run a cpu sample
$ ./sample
// run a gpu sample
$ ./sample_gpu
```
