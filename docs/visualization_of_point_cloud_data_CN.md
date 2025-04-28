# Visualization_of_point_cloud_data

## 1 简介
本文档展示如何可视化点云

## 2 设置参数及路径
进入 [pcl_tool.cc](..\tool\pcl_tool.cc) 

#### 2.1 确保ENABLE_VIEWER被定义
```cpp
#define ENABLE_VIEWER;//确保该行代码解注释
```

#### 2.2 可视化实时点云
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_LIDAR;//设置数据来源为实时
  param.input_param.ptc_port = 9347;//TCP端口号
  param.input_param.udp_port = 2368;//UDP端口号
  param.input_param.host_ip_address = "192.168.1.100";//终端地址
  param.input_param.firetimes_path = {"Your firetime file path"}; // 可选项
```
#### 2.3 可视化离线点云
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;//设置数据来源为离线数据
  param.input_param.pcap_path = {"Your pcap file path"}; // 离线数据路径
  param.input_param.correction_file_path = {"Your correction file path"}; //校准文件路径
  param.input_param.firetimes_path = {"Your firetime file path"}; // 可选项
```


## 3 编译
在tool文件夹路径下，创建build文件夹，在build文件夹下新启terminal，运行cmake ..指令，运行make指令
```cpp
cd HesaiLidar_SDK_2.0/tool
mkdir build
cd build
cmake ..
make
```

## 4 运行
成功编译后，build文件夹下会生成pcl_tool可执行文件，在build路径下输入./pcl_tool运行程序，系统会有可视化窗口
```cpp
./pcl_tool
```

