# 可视化点云数据
本文档展示如何利用PCL可视化点云

## 准备
进入 [pcl_tool.cc](../tool/pcl_tool.cc) 

### 启用点云查看器
```cpp
#define ENABLE_VIEWER;  // 确保该行代码解注释
```

##### 可视化实时点云
```cpp
// 设置参数
param.input_param.source_type = DATA_FROM_LIDAR;  // 设置数据来源为实时数据
param.input_param.ptc_port = 9347;  // TCP端口号
param.input_param.udp_port = 2368;  // UDP端口号
param.input_param.host_ip_address = "192.168.1.100";  //本地网口IP
param.input_param.correction_file_path = {"Your correction file path"};   // 校准文件（角度修正文件），建议使用雷达自身校准文件
param.input_param.firetimes_path = {"Your firetime file path"};   // 可选项：通道发光时序（发光时刻修正文件）
```

##### 可视化离线点云
```cpp
// 设置参数
param.input_param.source_type = DATA_FROM_PCAP; // 设置数据来源为离线PCAP数据
param.input_param.pcap_path = {"Your pcap file path"};  // 离线数据路径
param.input_param.correction_file_path = {"Your correction file path"};   // 校准文件（角度修正文件），建议使用雷达自身校准文件
param.input_param.firetimes_path = {"Your firetime file path"}; // 可选项：通道发光时序（发光时刻修正文件）
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
成功编译后，在build文件夹下运行生成的pcl_tool可执行文件，系统会有可视化窗口。
```cpp
./pcl_tool
```

