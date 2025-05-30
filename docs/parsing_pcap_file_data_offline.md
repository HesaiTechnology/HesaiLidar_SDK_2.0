# Parsing PCAP File Data Offline

## Preparation
```cpp
// test.cc - PCAP configuration
param.input_param.source_type = DATA_FROM_PCAP;
param.input_param.pcap_path = "path/to/pcap";              // Path to recorded pcap file
param.input_param.correction_file_path = "/path/to/correction.csv";  // Calibration file path (Angle Correction file path), it is recommended to use the angle correction file provided by the lidar itself
param.input_param.firetimes_path = "path/to/firetimes.csv";          // Laser firing sequence (Firetimes file path)
param.decoder_param.distance_correction_lidar_flag = false; // Set to true when distance correction needs to be turned on
```

## Steps
```bash
# 1. Build Sample (from build directory)
make -j$(nproc)

# 2. Run Sample
$ ./sample
```