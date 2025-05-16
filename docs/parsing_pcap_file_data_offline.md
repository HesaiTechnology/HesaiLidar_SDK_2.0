# Parsing PCAP File Data Offline

## Preparation
#### Use CPU parsing
```cpp
// test.cc - PCAP configuration
param.input_param.source_type = DATA_FROM_PCAP;
param.input_param.pcap_path = "path/to/pcap";              // Path to recorded pcap file
param.input_param.correction_file_path = "/path/to/correction.csv";  // Calibration file path (Angle Correction file path)
param.input_param.firetimes_path = "path/to/firetimes.csv";          // Laser firing sequence (Firetimes file path)
param.decoder_param.distance_correction_lidar_flag = false; // Set to true when distance correction needs to be turned on
```

#### Use GPU parsing
##### 1 Enable GPU parsing
```cpp
// CMakeLists.txt - Remove code comments
find_package(CUDA )
if(${CUDA_FOUND})
```

##### 2 PCAP confuguration
```cpp
// test.cu - PCAP configuration
param.input_param.source_type = DATA_FROM_PCAP;
param.input_param.pcap_path = "path/to/pcap";              // Path to recorded pcap file
param.input_param.correction_file_path = "/path/to/correction.csv";  // Calibration file path (Angle Correction file path)
param.input_param.firetimes_path = "path/to/firetimes.csv";          // Laser firing sequence (Firetimes file path)
param.decoder_param.distance_correction_lidar_flag = false; // Set to true when distance correction needs to be turned on
```

## Steps
#### Use CPU parsing
```bash
# 1. Build Sample (from build directory)
make -j$(nproc)

# 2. Run Sample
$ ./sample
```

#### Use GPU parsing
```bash
# 1. Build Sample (from build directory)
make -j$(nproc)

# 2. Run Sample
$ ./sample_gpu
```