# HesaiLidar_SDK_2.0

################################################################################
Friday, June 9th, 2023 16:45 
## version
V2.0.1

## modify
1. first update
2. fix AT128 frame segmentation bug

################################################################################
Monday, October 16th, 2023 11:00 
## version
V2.0.4

## modify
1. support ET25 

################################################################################
Wednesday, October 25th, 2023 20:00 
## version
V2.0.5

## modify
1. Fix bug in decode OT128 timestamp

################################################################################
Monday, January 15th, 2024 21:30
## version
V2.0.6

## modify
1. Add point cloud timestamp usage type configuration
2. Add ptc connection configuration parameters
3. Add et angle global correction
4. Fix the problem that coordinate transformation parameters do not take effect.
5. Added unified memory management in the LidarDecodedFrame class
6. Support P128 and XT32 to set the speed and standby through the configuration file
7. Support the configuration of common radar configuration items
8. Add support for AT128 to intercept FOV display


################################################################################
Saturday April 13th, 2024 20:10:00 CST
## version
V2.0.7

## modify
1. Fix gpu compile problem
2. Resolve compile warnings, inluding cpu compile and gpu compile
3. ROS add topics, support angle correction data, ptp data, loss bag count data
4. Support AT128P inversion
5. Update XT16/XT32/XT32M2X/OT128 firetime correction file
6. Update AT128 and AT128P angle correction file
7. Resolve windows compile error