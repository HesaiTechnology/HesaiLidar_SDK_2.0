# HesaiLidar_SDK_2.0

[English Version](<README.md>)

## 1 适用性

### 1.1 支持的雷达型号
- Pandar40P
- Pandar64
- Pandar128E3X
- OT128
- PandarQT
- QT128C2X
- PandarXT-16
- PandarXT
- XT32M2X
- AT128E2X
- AT128P
- ATX
- FT120
- ET25
- ETX
- JT16
- JT128

### 1.2 支持的操作系统
- Ubuntu 16.04
- Ubuntu 18.04
- Ubuntu 20.04
- Ubuntu 22.04
- Windows 10

## 2 克隆 & 编译

### 2.1 克隆
```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git
```
**注意:** 我们不建议在Windows上使用压缩包，因为它可能会由于符号问题导致编译错误。

### 2.2 编译器 & 依赖项 
**编译器版本要求**
- Ubuntu 16.04 及以上
	- Cmake 版本要求: Cmake 3.8.0 及以上
	- G++ 版本要求: G++ 7.5 及以上
- Windows 10
	- Cmake 版本要求: Cmake 3.8.0 及以上
	- MSVC 版本要求: MSVC 2019 及以上

**依赖项**
- `PCL` (如果使用点云可视化功能，需安装PCL)
- `libpcap` (如果解析PCAP文件，需安装libpcap)
- `libyaml` (如果解析雷达修正文件，需安装libyaml)

### 2.3 编译

#### 2.3.1 Ubuntu下的编译说明
```bash
# 0. 安装依赖项
sudo apt update && sudo apt install -y libpcl-dev libpcap-dev libyaml-cpp-dev

# 1. 导航到源目录
cd HesaiLidar_SDK_2.0

# 2. 创建build目录并导航到build目录
mkdir -p build && cd build

# 3. 使用Cmake配置项目
#    - 添加 -DCMAKE_BUILD_TYPE=Release 用于优化编译
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. 编译SDK
#    - 使用 -j$(nproc) 来利用所有的CPU内核
make -j$(nproc)
```

#### 2.3.2 Windows下的编译说明
请参考 **[如何在Windows下编译SDK](docs\compile_on_windows_CN.md)**.

## 3 应用指南

### 3.1 在线解析激光雷达数据
 请参考 **[如何在线解析激光雷达数据](docs\parsing_lidar_data_online_CN.md)**。

### 3.2 离线解析PCAP文件数据
请参考 **[如何离线解析PCAP文件数据](docs\parsing_pcap_file_data_offline_CN.md)**。

### 3.3 点云数据的可视化
请参考 **[如何可视化点云数据](docs\visualization_of_point_cloud_data_CN.md)**。

### 3.4 在线解析多激光雷达数据
请参考 **[如何在线解析多激光雷达数据](docs\parsing_multi_lidar_data_online_CN.md)**。

### 3.5 离线解析多激光雷达数据
请参考 **[如何离线解析PCAP中多激光雷达的数据](docs\parsing_multi_lidar_data_offline_CN.md)**。

### 3.6 保存点云数据为PCAP文件
请参考 **[如何将点云数据保存为PCAP文件](docs\save_point_cloud_data_as_a_pcap_file_CN.md)**。

### 3.7 坐标转换
请参考 **[如何进行坐标转换](docs\coordinate_transformation_CN.md)**。

### 3.8 定义帧时间戳
请参考 **[如何定义帧时间戳](docs\define_frame_timestamp_CN.md)**。

### 3.9 保存点云数据为PCD文件
请参考 **[如何将点云数据保存为PCD文件](docs\save_point_cloud_data_as_a_pcd_file_CN.md)**。

### 3.10 使用GPU加速
请参考 **[如何使用GPU加速优化性能](docs\use_gpu_acceleration_CN.md)**.

### 3.11 调用SDK API命令接口
请参考 **[如何调用SDK API命令接口](docs\invoke_sdk_api_command_interface_CN.md)**.

### 3.12 常见故障处理（WARNING）
请参考 **[常见故障处理（WARNING）](docs\common_troubleshooting_CN.md)**.

### 3.13 丢包统计
请参考 **[如何进行丢包统计](docs\packet_loss_analysis_CN.md)**.

## 4 功能参数解释
请参考 **[功能参数解释](docs\parameter_introduction_CN.md)**.