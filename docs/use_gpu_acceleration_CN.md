# 使用GPU解析
通过将点云解析任务迁移到GPU上，不仅可以释放CPU的计算资源，在一定程度上还能有效提升点云数据解析的速度与效率。


## 准备
使用GPU解析点云需要安装好显卡驱动及CUDA环境。
1. 终端输入`nvidia-smi`命令可以看到当前GPU信息，若未安装可参考：[NVIDIA-DRIVERS](https://www.nvidia.cn/drivers/)
2. 终端输入`nvcc -V`命令可以看到当前CUDA版本，若未安装可参考：[CUDA-DOWNLOADS](https://developer.nvidia.com/cuda-downloads)

#### SDK配置
在HesaiLidar_SDK_2.0文件夹下[CMakeLists.txt](../CMakeLists.txt) 文件中，确认用于查找和配置 NVIDIA CUDA 工具包的命令已释放：
```cpp
find_package(CUDA)
```

#### 1 使用CUDA解析实时数据
```cpp
  // 进入 test.cu 进行网络配置
  param.input_param.source_type = DATA_FROM_LIDAR;  // 数据来源是实时数据
  param.input_param.device_ip_address = "192.168.1.201"; // 雷达IP
  param.input_param.ptc_port = 9347;  // TCP端口
  param.input_param.udp_port = 2368;  // UDP端口
  param.input_param.host_ip_address = ""; // 本地网口IP（可选填）
  param.input_param.multicast_ip_address = "";  // 组播IP（如有，必填）
  param.decoder_param.distance_correction_lidar_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;
  ```
#### 2 使用CUDA解析离线数据
```cpp
  // 进入 test.cu 进行网络配置
  param.input_param.source_type = DATA_FROM_PCAP; // 数据来源是PCAP点云数据
  param.input_param.pcap_path = {"Your pcap file path"}; // PCAP文件路径
  param.input_param.correction_file_path = {"Your correction file path"}; // 角度修正文件（必填项）
  param.input_param.firetimes_path = {"Your firetime file path"}; // 发光时刻修正文件（选填项）
   ```


## 操作
### 1 编译
在HesaiLidar_SDK_2.0文件夹下，启动Terminal终端，执行以下指令。
```bash
cd HesaiLidar_SDK_2.0
mkdir -p build 
cd build
cmake ..
make
```

### 2 运行
成功编译后，在build文件夹下运行生成的sample_gpu可执行文件。
```bash
./sample_gpu
```


## 更多参考
如果安装了多个CUDA版本，可以在[CMakeLists.txt](../CMakeLists.txt) 指定CUDA版本路径:

```cpp
   # if install different cuda version, set the cuda path, like cuda-11.4
   # set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
   ```
