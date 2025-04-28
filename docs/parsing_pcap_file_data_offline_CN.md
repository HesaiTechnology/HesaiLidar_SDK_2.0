# Parsing_pcap_file_data_offline

## 1 简介
本文档展示如何解析离线数据

## 2 设置参数及路径
### 2.1 CPU解析
进入 [test.cc](..\test\test.cc) 

#### 2.1.1 设置正确的参数
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;//设置数据来源为离线数据
  param.input_param.pcap_path = {"Your pcap file path"}; // 离线数据路径
  param.input_param.correction_file_path = {"Your correction file path"}; //校准文件路径
  param.input_param.firetimes_path = {"Your firetime file path"}; // 可选项
```

### 2.2 GPU解析

#### 2.2.1 确保GPU解析功能开启
进入[CMakeLists.txt](..\CMakeLists.txt)
```cpp
if(${CUDA_FOUND}) ;//确保该行代码解除注释
```
#### 2.2.2 设置正确的参数
进入 [test.cu](..\test\test.cu) 
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;//设置数据来源为离线数据
  param.input_param.pcap_path = {"Your pcap file path"}; // 离线数据路径
  param.input_param.correction_file_path = {"Your correction file path"}; //校准文件路径
  param.input_param.firetimes_path = {"Your firetime file path"}; // 可选项
```

## 3 编译
在Hesai_SDK_2.0文件夹路径下，创建build文件夹，在build文件夹下新启terminal，运行cmake ..指令，运行make指令
```cpp
cd HesaiLidar_SDK_2.0
mkdir build
cd build
cmake ..
make
```

## 4 运行

### 4.1 CPU解析
成功编译后，build文件夹下会生成sample可执行文件，在build路径下输入./sample运行程序，系统会开始解析数据
```cpp
./sample
```
### 4.2 GPU解析
成功编译后，build文件夹下会生成sample_gpu可执行文件，在build路径下输入./sample_gpu运行程序，系统会开始解析数据
```cpp
./sample_gpu