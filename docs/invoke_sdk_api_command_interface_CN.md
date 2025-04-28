# 发送PTC指令 

## 1 概述
本功能实现了通过SDK向雷达发送PTC指令，获取雷达的信息或者设置雷达参数

## 2 发送PTC指令的方式
在tool文件夹的ptc_tool.cc中param部分，设置好网络配置（默认的雷达设置状态下无需更改）
```cpp
param.input_param.device_ip_address = "192.168.1.201"; //雷达ip
param.input_param.ptc_port = 9347;  //无需更改             
param.input_param.udp_port = 2368;  //udp端口
param.input_param.host_ip_address = "192.168.1.100";  //上位机ip
```
### 编译
```bash
# 0. 安装依赖项
sudo apt update && sudo apt install -y libpcl-dev libpcap-dev libyaml-cpp-dev

# 1. 导航到源目录
cd HesaiLidar_SDK_2.0/tool

# 2. 创建build目录并导航到build目录
mkdir -p build && cd build

# 3. 使用Cmake配置项目
cmake ..

# 4. 编译SDK
make
```
### 运行
```bash
./ptc.tool
```

### 使用示例
#### 1. 设置雷达目的ip
设置目的ip和端口号
```cpp
std::string destination_ip = "255.255.255.255";
uint16_t udp_port = 2368;
uint16_t gps_udp_port = 10110;
```
运行后终端输出如下信息代表设置成功
```log
SetDesIpandPort successed!
Current destination_ip: 255.255.255.255, Current udp_port: 2368, Current gps_udp_port: 10111
```

#### 2. 设置雷达ip
将is_set_net设置为1
```cpp
 int is_set_net = 1;
```
设置ip、子网掩码、网关和vlan
```cpp
std::string net_IP = "192.168.1.202";
std::string net_mask = "255.255.255.0";
std::string net_getway = "192.168.1.1";
uint8_t vlan_flag = 0;
uint16_t vlan_ID = 0;
```
运行后终端输出如下信息代表设置成功
```log
SetNet successed!
Current net_IP: 192.168.1.202, Current net_mask: 255.255.255.0, Current net_getway: 192.168.1.1
Current vlan_flag: 0, Current vlan_ID: 0
```
注意：设置雷达ip后运行会终止，需要重新设置网络配置为雷达对应的ip后，修改is_set_net为0，再继续进行其它操作

#### 3. 获取雷达角度校准文件
在ptc.tool.cc中添加如下代码，会在build目录下生成correction文件
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
运行后终端输出如下信息代表设置成功
```log
Read correction file from lidar success
GetCorrectionInfo succeeded!
Correction data size: 526 bytes
Saved correction data to correction
```