# Convert PCAP to PCD
HesaiLidar_SDK_2.0 provides sample code for converting PCAP format files to PCD format point clouds.

## Preparation
Open [pcl_tools.cc](../tool/pcl_tool.cc) 

#### 1 Assign param and path
```cpp
  // assign param
  param.input_param.source_type = DATA_FROM_PCAP;                         // Set the data source to PCAP data
  param.input_param.pcap_path = {"Your pcap file path"};                  // PCAP file path
  param.input_param.correction_file_path = {"Your correction file path"}; // Calibration file path (Angle Correction file path)
  param.input_param.firetimes_path = {"Your firetime file path"}; // Optional：Laser firing sequence (Firetimes file path)
```

#### 2 Uncomment the macro definition
The sample code contains four commented-out macro definitions (`#define`), whose purpose is to control the enabling or disabling of the function for saving different types of PCD point cloud formats in the program:
```cpp
#define SAVE_PCD_FILE_ASCII
// #define SAVE_PCD_FILE_BIN
// #define SAVE_PCD_FILE_BIN_COMPRESSED
// #define SAVE_PLY_FILE
```
1. **`SAVE_PCD_FILE_ASCII`**  
   Enabling this macro saves PCD files in ASCII format—a readable text format useful for debugging, though with larger file sizes and slower I/O performance.

2. **`SAVE_PCD_FILE_BIN`**  
   Enabling this macro saves PCD files in binary format, which offers greater efficiency than ASCII with smaller sizes and faster I/O, though lacking human-readability.

3. **`SAVE_PCD_FILE_BIN_COMPRESSED`**  
   Enabling this macro may cause the program to save PCD files using compressed binary format, which applies additional compression to binary data to achieve reduced storage requirements.

4. **`SAVE_PLY_FILE`**  
   Enabling this macro may output PLY files (Polygon File Format), a standard format for 3D data storage.


## Steps
### 1 Compile
Navigate to the HesaiLidar_SDK_2.0 directory, open a Terminal window, and run the following commands.
```bash
cd HesaiLidar_SDK_2.0/tool
mkdir build
cd build
cmake ..
make
```

### 2 Run
Once compiled successfully, go to the build folder, execute the generated pcl_tool executable, and the system will display a visualization window. The system will also generate per-frame point cloud data from the corresponding PCAP file in the build directory.
```bash
./pcl_tool
```
**Note：**
In case of frame drops in the output PCD files, adjust the delay parameter at the end of pcl_tool.cc by increasing it from the default 40ms to a higher value (1000ms recommended for testing).
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(1000))
```

## Reference
#### 1 Data Precision Configuration in ASCII PCD Files
The `writeASCII` method (in `pcl::PCDWriter`) is typically used as follows:
```cpp
pcl::PCDWriter writer;
writer.writeASCII("output.pcd", cloud);
```
**Note:** 
By default, the PCD writer saves data with limited precision - timestamps are stored in scientific notation, for instance.

**Configuring Precision Levels for PCD File Generation**
```cpp
pcl::PCDWriter writer;
// you can change the value of precision to adjust the precison
int precision = 16;
writer.writeASCII(file_name1, *pcl_pointcloud, precision);
```

#### 2 Configuring Timestamp Formats for PCD Filenames
- The PCD filename's timestamp corresponds to the first point's timestamp in the frame header.
```cpp
  std::string file_name1 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[0].timestamp)+ ".pcd";
  std::string file_name2 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[0].timestamp)+ ".bin";
  std::string file_name3 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[0].timestamp)+ ".ply";
  std::string file_name4 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[0].timestamp)+ "_compress" + ".bin";
```

- The PCD filename's timestamp corresponds to the last point's timestamp in the frame.
```cpp
  std::string file_name1 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[points_num - 1].timestamp)+ ".pcd";
  std::string file_name2 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[points_num - 1].timestamp)+ ".bin";
  std::string file_name3 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[points_num - 1].timestamp)+ ".ply";
  std::string file_name4 = "./PointCloudFrame" + std::to_string(frame.frame_index) + "_" + std::to_string(frame.points[points_num - 1].timestamp)+ "_compress" + ".bin";
```