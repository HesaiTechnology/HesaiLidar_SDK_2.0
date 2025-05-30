# Use GPU Acceleration
By transferring the point cloud parsing task to the GPU, not only can the computing resources of the CPU be released, but it can also effectively enhance the speed and efficiency of point cloud data parsing to a certain extent.


## Preparation
To parse point clouds using GPU, you need to install the graphics card driver and the CUDA environment.
1. Input `nvidia-smi` command and you can see the version of GPU. If not installed, please refer to [NVIDIA-DRIVERS](https://www.nvidia.cn/drivers/).
2. Input `nvcc -V` command and you can see the version of GPU. If not installed, please refer to [CUDA-DOWNLOADS](https://developer.nvidia.com/cuda-downloads).

#### SDK configuration
Open [CMakeLists.txt](../CMakeLists.txt) in HesaiLidar_SDK_2.0 directory and make sure that the commands used for searching and configuring the NVIDIA CUDA toolkit have been released:
```cpp
find_package(CUDA)
```

#### 1 Use CUDA to parse data online
```cpp
  // test.cu - Network configuration
  param.input_param.source_type = DATA_FROM_LIDAR; 
  param.input_param.device_ip_address = "192.168.1.201"; // IP address of the lidar
  param.input_param.ptc_port = 9347;  // TCP port
  param.input_param.udp_port = 2368;  // UDP port
  param.input_param.host_ip_address = ""; // Host PC IP address (Optional)
  param.input_param.multicast_ip_address = "";  // Multicast IP (If applicable, please fill in)
  param.decoder_param.distance_correction_lidar_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;
  ```
#### 2 Use CUDA to parse data offline
```cpp
  // test.cu - Network configuration
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = {"Your pcap file path"}; // the path of PCAP file
  param.input_param.correction_file_path = {"Your correction file path"}; // the path of angle correction file（required）
  param.input_param.firetimes_path = {"Your firetime file path"}; // the path of firetime file（optional）
   ```


## Steps
### 1 Compile
Navigate to the HesaiLidar_SDK_2.0 directory, open a Terminal window, and run the following commands.
```bash
cd HesaiLidar_SDK_2.0
mkdir -p build 
cd build
cmake ..
make
```

### 2 Run
Once compiled successfully, go to the build folder, execute the generated sample_gpu executable.
```bash
./sample_gpu
```


## Reference
If multiple CUDA versions are installed, the CUDA version path can be specified in [CMakeLists.txt](../CMakeLists.txt):

```cpp
   # if install different cuda version, set the cuda path, like cuda-11.4
   # set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
   ```
