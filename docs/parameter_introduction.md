# Functional Parameter Reference

### 1 [DecoderParam](../libhesai/driver_param.h)

#### 1.1 Time Synchronization
1. `pcap_play_synchronization`: Enable real-time playback simulation when parsing PCAP files
    *(Default: true)*

2. `use_timestamp_type`: Select timestamp source  
   - `0`: Point cloud timestamp (default)
   - `1`: System local time

#### 1.2 Frame Processing
1. `frame_start_azimuth`: Frame segmentation start angle (0-360 degrees)
    *(Default: 0.0)*

2. `fov_start`/`fov_end`: Valid field of view range (angle values), point clouds outside this range will be filtered  
   *(Default: 0.0-360.0)*

3. `channel_fov_filter_path`: Configuration file path (function: filter horizontal angles for multiple ranges under multiple channels), format reference: correction/config/channel_fov_filter.txt

4. `multi_fov_filter_ranges`: Compared with the channel_fov_filter_path parameter, this parameter has a similar function but targets all channels (i.e., filtering is applied to all channels). While this configuration method is more convenient, it lacks the flexibility of specifying individual channels. Its advantages include slightly lower CPU usage compared to channel_fov_filter_path and simpler configuration.
For example: "[20,30];[40,200]"This is a string configuration where each range is separated by ; and enclosed in []. The closed interval of the range is separated by a comma, such as "[20,30]"

5. `frame_frequency`: Frame frequency configuration, unit Hz. Cannot be configured to exceed the lidar's frequency. For example, if the lidar is 10Hz, it cannot be configured to a lidar frequency greater than 10Hz. Default is 0, which means no frame frequency is configured. Used in conjunction with default_frame_frequency.
   *(Default: 0)*

6. `default_frame_frequency`: Default frame frequency, this parameter defaults to 10.0, used in conjunction with the frame_frequency parameter. This value must be consistent with the actual frequency of the lidar. Frame rates other than 10.0 have detection prompts enabled in the code; continuous reminders indicate that the configured frame frequency is incorrect.
   *(Default: 10.0)* 

7. `play_rate_`: Controls the PCAP playback rate.
   *(Default: 1.0)*

8. `echo_mode_filter`: Filters specified echo data, 0 to get all echo data, n to get the nth type of echo data under the current echo mode.
   *(Default: 0)*

#### 1.3 Packet Analysis
1. `enable_packet_loss_tool`: Enable sequence number-based packet loss statistics  
   *(Default: false)*

2. `enable_packet_timeloss_tool`: Enable timestamp-based packet loss detection  
   *(Default: false)*

3. `packet_timeloss_tool_continue`: Continuously monitor timestamp continuity
   *(Default: false)*

4. `update_function_safety_flag`: Parses the functional safety field, access funcSafety to get the parsing result, supported by some mechanical lidars.
   *(Default: false)*

#### 1.4 Calibration Settings
1. `distance_correction_lidar_flag`: Enable optical center correction for mechanical lidars
   *(Optional for Pandar/QT/OT/XT/JT series)*

2. `xt_spot_correction`: Enable XT16/32 S-shaped layered correction  
   *(Default: false)*

#### 1.5 Performance
1. `system_udpsocket_buffer_size`: UDP socket buffer size (bytes) 
    *(Default: 1500)*

#### 1.6 Coordinate Transformation Related
1. `transform_param`: See definition [TransformParam](../libhesai/Common/include/hs_com.h), implements coordinate transformation

2. `remake_config`: See definition [RemakeConfig](../libhesai/Common/include/hs_com.h), implements data rearrangement according to horizontal and vertical angle positions

#### 1.7 Other Configurations
1. `pcap_play_in_loop`: Allow looping playback of PCAP data
      *(Default: false)*

2. `thread_num`: Multi-threaded parsing, single-threaded parsing when 1, multi-threaded parsing when greater than 1

3. `enable_udp_thread`: Whether to allow receiving point cloud data from lidar
      *(Default: true)*

4. `enable_parser_thread`: Whether to allow parser thread to work
      *(Default: true)*

5. `et_blooming_filter_flag`: Enable ET2.10 high-contrast blooming filter function
      *(Default: false)*

---

### 2 [InputParam](../libhesai/driver_param.h)

#### 2.1 Connection Basics
1. `source_type`: Data source selection  
   - `DATA_FROM_PCAP`: PCAP file playback
   - `DATA_FROM_LIDAR`: Network connection
   - `DATA_FROM_SERIAL`: Serial port connection

2. `ptc_mode`: Communication protocol mode  
   - `0`: TCP (default)
   - `1`: SSL encryption

#### 2.2 Network Configuration
1. `device_ip_address`: Lidar IP address
   *(Default: 192.168.1.201)*

2. `udp_port`: Point cloud data port  
   *(Default: 2368)*

3. `ptc_port`: Control protocol port  
   *(Default: 9347)*

4. `multicast_ip_address`: Multicast address 
   *(Empty string means no multicast address is set)*

5. `fault_message_port`: Fault message destination port number, configured when different from udp_port

6. `host_ip_address`: Host IP address

7. `device_udp_src_port`: Point cloud source port number, filter point clouds through point cloud source IP address + source port number, valid values [1024, 65535]

8. `device_fault_port`: Fault message source port number, filter fault messages through fault message source IP address + source port number, valid values [1024, 65535]

9. `use_ptc_connected`: Whether to use PTC communication

10. `host_ptc_port`: Specifies the source port number for PTC communication. When configured to 0, the system will randomly assign the source port number.


#### 2.3 File Paths
1. `pcap_path`: PCAP file path  
   *(Required for offline PCAP playback)*

2. `correction_file_path`: Calibration file path
   *(Default requires CSV format, some lidars support DAT format)*

3. `firetimes_path`: Firing timing correction file path

#### 2.4 Serial Port Configuration (JT16 only)
1. `rs485_com`: Point cloud receiving serial port  
    *(Example: COM3 or /dev/ttyUSB0)*

2. `rs232_com`: Command sending serial port

3. `point_cloud_baudrate`: RS485 baud rate  
    *(Default: 3125000)*

4. `rs485_baudrate`: OTA command baud rate
    *(Default: 115200)*

5. `rs232_baudrate`: Normal command baud rate  
    *(Default: 9600)*

6. `correction_save_path`: Local save path for angle calibration file obtained via serial port

#### 2.5 Security Configuration
1. `certFile`: Client certificate path  

2. `privateKeyFile`: Private key path 

3. `caFile`: CA certificate path 

#### 2.6 Lidar Configuration Related
1. `standby_mode`: Configure lidar working mode after successful PTC connection

2. `speed`: Configure lidar rotation speed after successful PTC connection

#### 2.7 Timeout Configuration
1. `recv_point_cloud_timeout`: Timeout for receiving point cloud data during initialization (s), effective when >= 0

2. `ptc_connect_timeout`: Timeout for PTC connection during initialization (s), effective when >= 0

---

### 3 [DriverParam](../libhesai/driver_param.h)

#### 3.1 Log Configuration
1. `log_level`: Log detail level  
   - `0`: Debug
   - `1`: Info (default)
   - `2`: Warning
   - `3`: Error

2. `log_Target`: Output target  
   - `0`: Running terminal (default)
   - `1`: File
   - `2`: Both outputs

3. `log_path`: Log file directory  
   *(Default: ./logs)*

4. `use_gpu`: Whether to use GPU for parsing