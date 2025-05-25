# 发送PTC指令 
本文档介绍了如何通过SDK向雷达发送PTC指令，获取雷达的信息或者设置雷达参数

## 准备
进入 [ptc_tool.cc](../tool/ptc_tool.cc) 
```cpp
// 网络配置（默认的雷达设置状态下无需更改）
param.input_param.device_ip_address = "192.168.1.201"; //雷达IP
param.input_param.ptc_port = 9347;  //TCP端口（无需更改）
param.input_param.udp_port = 2368;  //UDP端口
param.input_param.host_ip_address = "192.168.1.100";  //IP
```

## 操作
### 1 编译
在HesaiLidar_SDK_2.0文件夹下，启动Terminal终端，执行以下指令。
```cpp
cd HesaiLidar_SDK_2.0/tool
mkdir build
cd build
cmake ..
make
```

### 2 运行
成功编译后，在build文件夹下运行生成的ptc_tool可执行文件。
```cpp
./ptc_tool
```

## 更多参考
以下介绍几个常见的示例程序。
#### 1. 设置雷达目的IP
设置目的IP和相关端口号
```cpp
std::string destination_ip = "255.255.255.255";  //可以按需设置为单播、组播、广播
uint16_t udp_port = 2368;  //设置udp端口
uint16_t gps_udp_port = 10110;  //设置gps端口
```
运行后终端有如下相关打印代表设置参数成功
```log
SetDesIpandPort successed!
Current destination_ip: 255.255.255.255, Current udp_port: 2368, Current gps_udp_port: 10110
```

#### 2. 设置雷达IP
开启设置雷达IP的功能，将is_set_net设置为1
```cpp
 int is_set_net = 1;
```
设置IP、子网掩码、网关和VLAN
```cpp
std::string net_IP = "192.168.1.201";  //设置雷达IP
std::string net_mask = "255.255.255.0";  //设置子网掩码
std::string net_getway = "192.168.1.1";  //设置网关
uint8_t vlan_flag = 0;  //设置VLAN标志，0表示不设置，1表示设置
uint16_t vlan_ID = 0;  //设置VLAN ID
```
运行后终端有如下相关打印代表设置参数成功
```log
SetNet successed!
Current net_IP: 192.168.1.201, Current net_mask: 255.255.255.0, Current net_getway: 192.168.1.1
Current vlan_flag: 0, Current vlan_ID: 0
```
**注意：设置雷达IP后运行会终止，需要重新设置网络配置为雷达对应的IP后，修改is_set_net为0，再继续进行其它操作**

#### 3. 获取雷达角度校准文件
在[ptc_tool.cc](../tool/ptc_tool.cc)中添加如下代码，会在build目录下生成correction文件
```cpp
u8Array_t correction_data;
if (sample.lidar_ptr_->ptc_client_->GetCorrectionInfo(correction_data) == 0) {
    std::cout << "GetCorrectionInfo succeeded!" << std::endl;
    std::cout << "Correction data size: " << correction_data.size() << " bytes" << std::endl;

    // 如果你想保存到文件，比如保存为correction
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
运行后终端有如下相关打印代表设置参数成功
```log
Read correction file from lidar success
GetCorrectionInfo succeeded!
Correction data size: 526 bytes  // correction文件大小,不同雷达的大小不同
Saved correction data to correction
```