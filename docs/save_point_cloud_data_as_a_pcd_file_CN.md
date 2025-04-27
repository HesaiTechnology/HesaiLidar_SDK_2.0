# Convert PCAP to PCD

## 1 简介
本文档展示如何将PCAP文件转换为PCD格式的点云。

HesaiLidar_SDK_2.0提供了将PCAP文件转换为PCD格式的点云的示例代码。

## 2 设置参数及路径
进入 [pcl_tools.cc](..\tool\pcl_tool.cc) 找到main函数

#### 2.1 将source_type改为DATA_FROM_PCAP
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;
```
#### 2.2 将.PCAP和校准文件路径改为实际路径
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = {"Your pcap file path"}; // required
  param.input_param.correction_file_path = {"Your correction file path"}; //required
  param.input_param.firetimes_path = {"Your firetime file path"}; // optional
```

#### 2.3 将任一保存PCD点云的宏定义取消注释
示例代码中包含四个被注释掉的宏定义（`#define`），作用是控制程序转存不同类型PCD点云格式功能的启用或禁用：
1. **`SAVE_PCD_FILE_ASCII`**  
   启用这个宏定义，程序会以ASCII格式保存PCD文件。ASCII格式是一种文本格式，数据以可读的形式存储，便于调试和查看，但文件体积通常较大，读取和写入速度较慢。

2. **`SAVE_PCD_FILE_BIN`**  
   启用这个宏定义，程序会以二进制格式保存PCD文件。二进制格式相比ASCII格式更高效，文件体积更小，读写速度更快，但数据不可直接阅读。

3. **`SAVE_PCD_FILE_BIN_COMPRESSED`**  
   启用这个宏定义，程序可能会以压缩的二进制格式保存PCD文件。这种格式在二进制的基础上进一步压缩数据，进一步减小文件体积。

4. **`SAVE_PLY_FILE`**  
   启用这个宏定义后，程序可能会保存PLY文件（Polygon File Format或Stanford Triangle Format）。PLY文件是一种常用于存储三维数据的文件格式。
```cpp
#define SAVE_PCD_FILE_ASCII
// #define SAVE_PCD_FILE_BIN
// #define SAVE_PCD_FILE_BIN_COMPRESSED
// #define SAVE_PLY_FILE
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
成功编译后，build文件夹下会生成pcl_tool可执行文件，在build路径下输入./pcl_tool运行程序，程序会在build目录生成PCD
```cpp
./pcl_tool
```
如果生成的PCD缺帧，可以尝试加大pcl_tool.cc文件末尾的延时从40改大，比如1000ms
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(1000))
```

## 5 进阶应用
#### 5.1 如何定义ASCII格式的PCD文件数据精度

`writeASCII` 是 `pcl::PCDWriter` 类的一个方法，以下是其常见用法：
```cpp
pcl::PCDWriter writer;
writer.writeASCII("output.pcd", cloud);
```
按默认的方法保存的PCD文件数据精度有限，例如时间戳会表示为科学计数法。

如何更改PCD文件数据精度？
```cpp
pcl::PCDWriter writer;
// you can change the value of precision to adjust the precison
int precision = 16;
writer.writeASCII(file_name1, *pcl_pointcloud, precision);
```