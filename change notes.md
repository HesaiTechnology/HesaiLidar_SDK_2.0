# HesaiLidar_SDK_2.0

## V2.0.11

### Wednesday November 5th, 2025 16:30:00

### Added
1. Added function examples for backfilling point cloud data packets in EXTERNAL_INPUT_PARSER_TEST mode.
2. Added support for JT128 parsing.
3. Added support for parsing PCAP files with different definitions.
4. Added a new serial upgrade function for JT16, named RequestUpgradeLargePackage.
5. Added play_rate_ feature to control PCAP playback rate, with a default value of 1.0 representing 1x playback speed.
6. Added channel_fov_filter_path feature to configure FOV files for filtering point cloud data with multiple FOVs under multiple channels.
7. Added multi_fov_filter_ranges feature to configure FOV files for filtering point cloud data with multiple FOVs across all channels.
8. Added frame_frequency feature to configure point cloud publishing frequency, requiring manual configuration of default_frame_frequency to the actual point cloud publishing frequency.
9. Added host_ptc_port feature to configure the local port when connecting to PTC.
10. Added update_function_safety_flag feature; when enabled, function safety parsing results can be obtained by accessing the funcSafety struct (supported by some mechanical LiDARs).
11. Added echo_mode_filter feature to configure echo mode filtering, with a default value of 0 indicating no filtering.

### Fix
1. Modified the exported PCL struct definition in pcl_tool.cc to meet Windows compilation requirements.
2. Adjusted memory allocation for large data blocks in PcapSaver to occur only on first use, reducing memory overhead.

### Changed
1. ATX now uses the angle_division field from firetimes files to calculate firing times.
2. Revised GPU parsing logic for all LiDARs, copying raw data packets for a frame directly to GPU for parsing and computation, reducing data copy times and improving parsing efficiency.
3. Adjusted CPU parsing logic for all LiDARs: DecodePacket function is now responsible for frame separation, non-point-cloud information parsing, and timestamp parsing. ComputeXYZI function now handles point cloud information parsing, reordering, coordinate transformation, and other point cloud-related operations.
4. Removed PointDecodeData struct and the union data in PacketDecodeData struct to align with the CPU parsing logic adjustments.

## V2.0.10

### Monday June 30rd, 2025 17:28:15

### modify
1. Remove support for JT256, JT128, and ET.
2. Add the function to obtain precise timestamps for each point for Pandar series LiDAR, OT128, XT series LiDAR, and QT series LiDAR. Note: firetimes files must be loaded, see README for details.
3. When OT128 LiDAR encounters hash verification errors for angle calibration files, change from "error reporting + refusing to load" to only printing warnings.
4. Add the function to rearrange point clouds by horizontal and vertical angles, enabled through RemakeConfig configuration, see README for details.
5. Reduce the IMU transmission frequency for Pandar series LiDAR and OT128 LiDAR, modified to send only when IMU data changes.
6. Merge CPU and GPU code in the `driver`, with the ability to control whether to enable GPU parsing through switches, disabled by default, see README for details.
7. Add timeout exit logic for receiving point clouds and PTC connections, implemented through configuration parameters, see README for details.
8. Reduce the number of time calculation function calls to lower CPU usage.
9. Add compilation options to control the compilation of each module, add independent PTC examples, multi-LiDAR receiving examples, and examples for removing OpenSSL dependencies, see README for details.

## V2.0.9

### Monday December 23rd, 2024 14:57:29 CST

### modify
1. Support JT16, ET30HA2, JT128, JT256 parsing. (JT128/256 need to add macro definition JT128_256)
2. Distance correction only needs to be set on and off, no need to specify the lidar type again.
3. Support LAS protocol format storage.
4. The initialization distinguishes (PandarN, OT) (XT, XTM), and the point cloud S-stratification correction of XT is controlled using a flag bit parameter (xt_spot_correction).
5. Support ET2.5(ET25-E2X) 2.7(ET30HA2) for firetimes correction.
6. OT128 support to get weight factor and env light fields (Need to register its template as LidarPointXYZICWERT when declaring HesaiLidarSdk).

## V2.0.8

### Wednesday September 25th, 2024 11:04:33 CST

### modify
1. support ET25, ET30, ET30H, ATX parsing.
2. fix the problem that the length of point cloud and the parsing result are not checked, which leads to the problem that the number of packets and points in individual frames is too high.
3. support filtering point cloud and fault message data from specified source ports when parsing multiple lidar.
4. Modify the optical centre correction scheme of mechanical lidar, you can configure the corresponding parameters to enable the optical centre correction of different types of lidar.
5. fix the problem that the logger class starts repeatedly and causes a crash.
6. fix the problem that the shared data is not protected when multi-threaded parsing.
7. support XT16 and XT32 to correct the point cloud S stratification.
8. improve the GPU function of all models of lidar and optimise the copying logic.
9. support the parsing of AT128/AT128P and ATX's fault message.
10. support packet loss detection and point cloud timestamp fallback detection.
11. adjust the startup sequence, do not need to wait for the PTC to connect successfully before starting to parse the fault message.
12. remove LidarDecodedPacket type, use LidarDecodedFrame type to store intermediate results, and optimise the parsing process.
13. fix the bug of pcap reading overflow.
14. fix the problem that mechanical lidar can't be divided into frames after FOV is turned on.
15. use a separate thread for PTC initialisation, and lock when PTC is called to prevent data from being messed up.
16. Refuse to parse the point cloud when the angle calibration file is not loaded to prevent parsing data anomaly.
17. Use log module to record logs.
18. update Firetime files of OT128, Pandar40P, Pandar64, support Firetime files of ATX, PandarQT, and angle calibration files of ATX 4.2.
19. Fix the bug that the change of LidarPoint type to no timestamp leads to the exception of obtaining timestamp directly.
20. Compatible with OT firetimes analysis
21. Support reading ATX .dat format firetimes.
22. Fix the problem of not considering forward and reverse rotation when fixing mechanical lidar firetimes.


## V2.0.7

### Saturday April 13th, 2024 20:10:00 CST

### modify
1. Fix gpu compile problem
2. Resolve compile warnings, inluding cpu compile and gpu compile
3. ROS add topics, support angle correction data, ptp data, loss bag count data
4. Support AT128P inversion
5. Update XT16/XT32/XT32M2X/OT128 firetime correction file
6. Update AT128 and AT128P angle correction file
7. Resolve windows compile error


## V2.0.6 

### Monday, January 15th, 2024 21:30

## modify
1. Add point cloud timestamp usage type configuration
2. Add ptc connection configuration parameters
3. Add et angle global correction
4. Fix the problem that coordinate transformation parameters do not take effect.
5. Added unified memory management in the LidarDecodedFrame class
6. Support P128 and XT32 to set the speed and standby through the configuration file
7. Support the configuration of common lidar configuration items
8. Add support for AT128 to intercept FOV display


## V2.0.5 

### Wednesday, October 25th, 2023 20:00 

## modify
1. Fix bug in decode OT128 timestamp


## V2.0.4 

### Monday, October 16th, 2023 11:00 

### modify
1. support ET25 

## V2.0.1 

### Friday, June 9th, 2023 16:45 

### modify
1. first update
2. fix AT128 frame segmentation bug


