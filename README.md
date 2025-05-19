# HesaiLidar_SDK_2.0

[👉 中文版](<README_CN.md>)

## 1 Applicability

### 1.1 Supported lidars

| Pandar       | OT    | QT       | XT          | AT       | ET   | JT    |
|:-------------|:------|:---------|:------------|:---------|:-----|:------|
| Pandar40P    | OT128 | PandarQT | PandarXT    | AT128E2X | ET25 | JT16  |
| Pandar64     | -     | QT128C2X | PandarXT-16 | AT128P   | ETX  | JT128 |
| Pandar128E3X | -     | -        | XT32M2X     | ATX      | -    | -     |

### 1.2 Supported OS

- Ubuntu 16/18/20/22.04 
- Windows 10

### 1.3 Compiler version

Ubuntu
- Cmake 3.8.0 or above
- G++ 7.5 or above

Windows
- Cmake 3.8.0 or above
- MSVC 2019 or above

### 1.4 Dependencies

- To visualize point cloud data, install `PCL`.
- To parse PCAP file data, install `libpcap`.
<!-- - To parse the lidar correction files, install `libyaml`. // Needed when parsing config.yaml of ROS  -->

## 2 Getting started

### 2.1 Clone
```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git
```
> In Windows, downloading the repository as a ZIP file is not recommended, as it may lead to compilation errors due to symbolic link issues.

### 2.2 Build
<!-- TODO compile vs build -->

#### 2.2.1 Build in Ubuntu
```bash
# 0. Install dependencies
sudo apt update && sudo apt install -y libpcl-dev libpcap-dev libyaml-cpp-dev

# 1. Navigate to source directory
cd HesaiLidar_SDK_2.0

# 2. Create build directory and navigate to it
mkdir -p build && cd build

# 3. Configure project with CMake
#    - Add -DCMAKE_BUILD_TYPE=Release for optimized build
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. Compile the SDK
#    - Use -j$(nproc) to utilize all CPU cores
make -j$(nproc)
```

#### 2.2.2 Build in Windows
Please refer to **[compile on Windows](docs/compile_on_windows.md)**.

## 3 Application Guide

### 3.1 Parsing lidar Data Online
Please refer to **[Parsing lidar Data Online](docs/parsing_lidar_data_online.md)**.

### 3.2 Parsing PCAP File Data Offline
Please refer to **[Parsing PCAP File Data Offline](docs/parsing_pcap_file_data_offline.md)**.

### 3.3 Visualization of Point Cloud Data
Please refer to **[Visualization of Point Cloud Data](docs/visualization_of_point_cloud_data.md)**.

### 3.4 Coordinate Transformation
Please refer to **[Coordinate Transformation](docs/coordinate_transformation.md)**.

### 3.5 Save Point Cloud Data as a PCD File
Please refer to **[Save Point Cloud Data as a PCD File](docs/save_point_cloud_data_as_a_pcd_file.md)**.

### 3.6 Use GPU Acceleration
Please refer to **[Use GPU Acceleration](docs/use_gpu_acceleration.md)**.

### 3.7 Invoke SDK API command interface
Please refer to **[Invoke SDK API command interface](docs/invoke_sdk_api_command_interface.md)**.

### 3.8 Common Troubleshooting (Warning)
Please refer to **[Common Troubleshooting (Warning)](docs/common_error_codes.md)**.

### 3.9 Packet Loss Analysis
Please refer to **[Packet Loss Analysis](docs/packet_loss_analysis.md)**.


## 4 Functional Parameter Reference
Please refer to **[Functional Parameter Reference](docs/parameter_introduction.md)**.
