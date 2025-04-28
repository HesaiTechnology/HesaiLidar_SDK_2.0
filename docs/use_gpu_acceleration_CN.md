# Use GPU Acceleration


随着激光雷达线束及分辨率的提升，产生的数据量显著增加。GPU凭借其强大的并行计算能力和高带宽内存，能够高效处理这些大规模数据。通过将点云解析任务迁移到GPU上，不仅可以释放CPU的计算资源，还能有效提升点云数据解析的速度与效率。

## 1 CUDA环境准备
使用GPU解析点云需要安装好显卡驱动及CUDA环境。

1. 终端输入`nvidia-smi`命令可以看到当前GPU信息，若未安装可参考：https://www.nvidia.cn/drivers/
2. 终端输入`nvcc -V`命令可以看到当前CUDA版本，若未安装可参考：https://developer.nvidia.com/cuda-downloads

## 2 SDK配置
在HesaiLidar_SDK_2.0文件夹下[CMakeLists.txt](../CMakeLists.txt) 文件中，确认用于查找和配置 NVIDIA CUDA 工具包的命令已释放：
```cpp
find_package(CUDA)
```
### 2.1 使用CUDA解析实时数据
进入[test.cu](../test/test.cu)找到main函数：
#### 2.1.1 将source_type改为DATA_FROM_LIDAR
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_LIDAR;
```
#### 2.1.2 配置网络参数
```cpp
  param.input_param.device_ip_address = "192.168.1.201"; //雷达ip
  param.input_param.ptc_port = 9347;
  param.input_param.udp_port = 2368; //雷达Destination Port
  param.input_param.host_ip_address = "";
  param.input_param.multicast_ip_address = "";
  param.decoder_param.distance_correction_lidar_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;
  ```
### 2.2 使用CUDA解析离线数据
进入[test.cu](../test/test.cu)找到main函数：
#### 2.2.1 将source_type改为DATA_FROM_PCAP
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;
   ```

#### 2.2.2 将.PCAP和校准文件路径改为实际路径
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = {"Your pcap file path"}; // required
  param.input_param.correction_file_path = {"Your correction file path"}; //required
  param.input_param.firetimes_path = {"Your firetime file path"}; // optional
   ```

## 3 更多参考
如果安装了多个CUDA版本，可以在[CMakeLists.txt](../CMakeLists.txt) 指定CUDA版本路径:

```cpp
   # if install different cuda version, set the cuda path, like cuda-11.4
   # set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
   ```
