# Send PTC Command 
This document shows how to send PTC(Pandar TCP Command) to lidar by SDK, to get information from lidar or set parameters of lidar.


## Preparation
Open [ptc_tool.cc](../tool/ptc_tool.cc) 
```cpp
// Network Configuration (no need to change if parameters of lidar is default)
param.input_param.device_ip_address = "192.168.1.201"; // lidar IP
param.input_param.ptc_port = 9347;  // TCP protocol port
param.input_param.udp_port = 2368;  // UDP protocol port
param.input_param.host_ip_address = "192.168.1.100";  // Host PC IP address
```

## Steps
### 1 Compile
Navigate to the HesaiLidar_SDK_2.0 directory, open a Terminal window, and run the following commands.
```bash
cd HesaiLidar_SDK_2.0/tool
mkdir build
cd build
cmake ..
make
```

### 2 Run
Once compiled successfully, go to the build folder, execute the generated ptc_tool executable.
```bash
./ptc_tool
```

## Reference
The following are some sample programs.
#### 1. Set Lidar Destination IP
This sample is to introduce how to set lidar destination IP and port
```cpp
std::string destination_ip = "255.255.255.255";  // it can be set unicast, multicast or boardcast as needed
uint16_t udp_port = 2368;  // set UDP port
uint16_t gps_udp_port = 10110;  // set GPS UDP port
```
After running in Terminal, the following related prints on the terminal indicate that the parameter settings have been successful.
```log
SetDesIpandPort successed!
Current destination_ip: 255.255.255.255, Current udp_port: 2368, Current gps_udp_port: 10110
```

#### 2. Set Lidar IP
Enable the function of setting Lidar IP by setting is_set_net to 1.
```cpp
 int is_set_net = 1;
```
Set net_IP, net_mask, net_getway and etc.
```cpp
std::string net_IP = "192.168.1.201";  // set lidar IP
std::string net_mask = "255.255.255.0";  // set netmask
std::string net_getway = "192.168.1.1";  // set net gateway
uint8_t vlan_flag = 0;  // flag of setting vlan (0: disable, 1: enable)
uint16_t vlan_ID = 0;  // set vlan ID
```
After running in Terminal, the following related prints on the terminal indicate that the parameter settings have been successful.
```log
SetNet successed!
Current net_IP: 192.168.1.201, Current net_mask: 255.255.255.0, Current net_getway: 192.168.1.1
Current vlan_flag: 0, Current vlan_ID: 0
```
**Note： The terminal will stop running ./ptc_tool while lidar IP set successful. After resetting the network configuration to the lidar IP, modify is_set_net to 0, and then continue with other setting operations**

#### 3. Get Lidar Angle Correction File
Add the following codes in [ptc_tool.cc](../tool/ptc_tool.cc), build again and run ./ptc_tool, then it will generate the correction file which get from lidar in build directory.
```cpp
u8Array_t correction_data;
if (sample.lidar_ptr_->ptc_client_->GetCorrectionInfo(correction_data) == 0) {
    std::cout << "GetCorrectionInfo succeeded!" << std::endl;
    std::cout << "Correction data size: " << correction_data.size() << " bytes" << std::endl;

    // if you wanna save as file, such as saving as correction
    FILE* fp = fopen("correction", "wb");
    if (fp != nullptr) {
        fwrite(correction_data.data(), 1, correction_data.size(), fp);
        fclose(fp);
        std::cout << "Saved correction data to correction" << std::endl;
    } else {
        std::cout << "Failed to open file for writing correction data!" << std::endl;
    }
} else {
    std::cout << "GetCorrectionInfo failed!" << std::endl;
}
```
After running in Terminal, the following related prints on the terminal indicate that the parameter settings have been successful.
```log
Read correction file from lidar success
GetCorrectionInfo succeeded!
Correction data size: 526 bytes  // The size of the correction file varies among different lidars
Saved correction data to correction
```