# Parsing Lidar data online

## Preparation
#### Network-connected lidars
```cpp
// test.cc - Network configuration
param.input_param.source_type = DATA_FROM_LIDAR;
param.input_param.device_ip_address = '192.168.1.201';     // IP address of the lidar
param.input_param.ptc_port = 9347;                         // TCP protocol port
param.input_param.udp_port = 2368;                         // Point cloud data port
param.input_param.host_ip_address = "192.168.1.100";       // Host PC IP address
param.input_param.multicast_ip_address = "239.0.0.1";      // Multicast group (if used)
param.input_param.correction_file_path = "/path/to/correction.csv";
param.decoder_param.distance_correction_lidar_flag = false;	// Set to true when distance correction needs to be turned on
```

#### Serial-connected lidars (JT16)
```cpp
// test.cc - Serial configuration
param.input_param.source_type = DATA_FROM_SERIAL;
param.input_param.rs485_com = "/dev/ttyUSB0";                   // RS485 port for point cloud (COM[X] in Windows)
param.input_param.rs232_com = "/dev/ttyUSB1";                   // RS232 port for commands (COM[Y] in Windows)
param.input_param.correction_file_path = "/path/to/correction.csv";
param.decoder_param.distance_correction_lidar_flag = false;	// Set to true when distance correction needs to be turned on
```

## Steps
```bash
# 1. Build Sample (from SDK root directory)
make -j$(nproc)

# 2. Run Sample
$ ./sample
```