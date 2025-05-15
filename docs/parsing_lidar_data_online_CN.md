# 在线解析雷达数据

## 准备

#### 以太网连接式激光雷达
```cpp
// 进入 test.cc 进行网络配置
param.input_param.source_type = DATA_FROM_LIDAR;           // 设置数据来源为在线点云数据
param.input_param.device_ip_address = '192.168.1.201';     // 激光雷达IP
param.input_param.ptc_port = 9347;                         // PTC协议端口
param.input_param.udp_port = 2368;                         // 点云数据端口
param.input_param.host_ip_address = "192.168.1.100";       // 主机网口IP
param.input_param.multicast_ip_address = "239.0.0.1";      // 可选项：如果目的IP为组播IP
param.input_param.correction_file_path = "/path/to/correction.csv";
param.decoder_param.distance_correction_lidar_flag = false;	// 可选项：当需要使用光心修正的时候设置为 true
```

#### 串口连接式激光雷达 (JT16)
```cpp
// 进入 test.cc 进行串口配置
param.input_param.source_type = DATA_FROM_SERIAL;               // 设置数据来源为串口数据
param.input_param.rs485_com = "/dev/ttyUSB0";                   // RS485 点云串口 (在Windows系统中是COM[X])
param.input_param.rs232_com = "/dev/ttyUSB1";                   // RS232 控制串口 (在Windows系统中是COM[Y])
param.input_param.correction_file_path = "/path/to/correction.csv";
param.decoder_param.distance_correction_lidar_flag = false;	// 可选项：当需要使用光心修正的时候设置为 true
```

## 操作
```bash
# 1. 构建可执行示例程序 (从build文件夹下)：成功编译后，生成可执行程序
make -j$(nproc)

# 2. 执行示例程序：开始解析数据
$ ./sample
```