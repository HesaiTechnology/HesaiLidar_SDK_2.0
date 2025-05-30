# Functional Parameter Reference

### 1 Decoder Parameters (DecoderParam)

#### 1.1 Time Synchronization
1. `pcap_play_synchronization`: Enable real-time playback simulation when parsing PCAP files  
   *(Default: true)*

2. `use_timestamp_type`: Select timestamp source  
   - `0`: Point cloud timestamp (default)
   - `1`: System local time

#### 1.2 Frame Processing
3. `frame_start_azimuth`: Frame split position (0-360 degrees)  
   *(Default: 0.0)*

4. `fov_start`/`fov_end`: Valid field-of-view range (degrees)  
   Points outside this range will be filtered  
   *(Default: 0.0-360.0)*

#### 1.3 Packet Analysis
5. `enable_packet_loss_tool`: Enable sequence-based packet loss statistics  
   *(Default: false)*

6. `enable_packet_timeloss_tool`: Enable timestamp-based packet loss detection  
   *(Default: false)*

7. `packet_timeloss_tool_continue`: Continuous timestamp monitoring  
   *(Default: false)*

#### 1.4 Calibration Settings
8. `distance_correction_lidar_flag`: Enable optical center correction for mechanical lidar models
   *(Required for Pandar/QT/OT/XT series)*

9. `xt_spot_correction`: Enable XT16/32 S-stratification correction  
   *(Default: false)*

#### 1.5 Performance
10. `system_udpsocket_buffer_size`: UDP socket buffer size (bytes)  
    *(Default: 1500)*

---

### 2 Input Parameters (InputParam)

#### 2.1 Connection Basics
1. `source_type`: Data source selection  
   - `DATA_FROM_PCAP`: PCAP file playback
   - `DATA_FROM_LIDAR`: Network connection
   - `DATA_FROM_SERIAL`: Serial connection

2. `ptc_mode`: Communication protocol mode  
   - `0`: TCP (default)
   - `1`: SSL encrypted

#### 2.2 Network Configuration
3. `device_ip_address`: IP address of the lidar
   *(Default: 192.168.1.201)*

4. `udp_port`: Point cloud data port  
   *(Default: 2368)*

5. `ptc_port`: Control protocol port  
   *(Default: 9347)*

6. `multicast_ip_address`: Multicast group address  
   *(Empty string for unicast)*

7. `host_ip_address`: Host computer IP address

#### 2.3 File Paths
8. `pcap_path`: PCAP file location  
   *(Required for PCAP mode)*

9. `correction_file_path`: Calibration file path  
   *(CSV format required)*

10. `firetimes_path`: Laser firing sequence file

#### 2.4 Serial Configuration (JT16 Only)
11. `rs485_com`: Point cloud reception port  
    *(e.g., COM3 or /dev/ttyUSB0)*

12. `rs232_com`: Command transmission port

13. `point_cloud_baudrate`: RS485 baud rate  
    *(Default: 3000000)*

14. `rs485_baudrate`: OTA command baud rate  
    *(Default: 115200)*

15. `rs232_baudrate`: Normal command baud rate  
    *(Default: 9600)*

#### 2.5 Security
16. `certFile`: Client certificate path  
17. `privateKeyFile`: Private key path  
18. `caFile`: CA certificate path  

---

### 3 Driver Parameters (DriverParam)

#### 3.1 Logging Configuration
1. `log_level`: Verbosity level  
   - `0`: Debug
   - `1`: Info (default)
   - `2`: Warning
   - `3`: Error

2. `log_Target`: Output destination  
   - `0`: Console (default)
   - `1`: File
   - `2`: Both

3. `log_path`: Log file directory  
   *(Default: ./logs)*