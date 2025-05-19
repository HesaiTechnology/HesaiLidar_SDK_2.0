# 离线解析PCAP点云数据

## 准备
```cpp
// 进入 test.cc 进行 PCAP 相关配置
param.input_param.source_type = DATA_FROM_PCAP;            // 设置数据来源为离线PCAP点云数据
param.input_param.pcap_path = "path/to/pcap";              // 离线PCAP点云数据路径
param.input_param.correction_file_path = "/path/to/correction.csv";  // 校准文件（角度修正文件），建议使用雷达自身的校准文件
param.input_param.firetimes_path = "path/to/firetimes.csv";          // 可选项：通道发光时序（发光时刻修正文件）
param.decoder_param.distance_correction_lidar_flag = false; // 可选项：当需要使用光心修正的时候设置为 true
```

## 操作
```bash
# 1. 构建可执行示例程序 (从build文件夹下)：成功编译后，生成可执行程序
make -j$(nproc)

# 2. 执行示例程序：开始解析数据
$ ./sample
```