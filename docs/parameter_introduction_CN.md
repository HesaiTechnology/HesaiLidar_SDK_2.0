# 功能参数参考

### 1 解码器参数 [DecoderParam](../libhesai/driver_param.h)

#### 1.1 时间同步
1. `pcap_play_synchronization`: 解析PCAP文件时启用实时播放模拟
    *(默认: true)*

2. `use_timestamp_type`: 选择时间戳来源  
   - `0`: 点云时间戳 (默认)
   - `1`: 系统本地时间

#### 1.2 帧处理
1. `frame_start_azimuth`: 帧分割起始角度 (0-360度)
    *(默认: 0.0)*

2. `fov_start`/`fov_end`: 有效视场范围 (角度值)，此范围外的点云将被过滤  
   *(默认: 0.0-360.0)*

3. `channel_fov_filter_path`: 配置文件路径(功能:对多组通道下多组范围的水平角进行滤除)，格式参考correction/config/channel_fov_filter.txt

4. `multi_fov_filter_ranges`: 相比较于channel_fov_filter_path参数，该参数与channel_fov_filter_path参数功能类似，但针对的是所有的通道。即对所有通道进行滤除。此处配置用法较为方便，但没有指定通道灵活。好处是cpu占用会比channel_fov_filter_path少一些，且配置简单些。比如"[20,30];[40,200]"。为字符串配置，每个范围用";"隔开，每个范围用"[]"括起来，范围的开闭区间中间用逗号隔开，如"[20,30]"。

5. `frame_frequency`: 帧频率配置，单位Hz。不能配置超过雷达的频率。比如雷达10Hz,无法配置成大于10Hz的雷达频率。默认0，即不配置帧频率。配合default_frame_frequency使用。
    *(默认: 0)* 

6. `default_frame_frequency`: 默认帧频率，此参数默认为10.0, 配合frame_frequency参数使用。该值必须和雷达的实际频率一致。非10.0的帧率在代码中开启了检测提示，当一直提醒说明配置的帧频率有误。
    *(默认: 10.0)* 

7. `play_rate_`: 控制pcap的播放速率
    *(默认: 1.0)*

8. `echo_mode_filter`: 过滤指定回波数据，0为获取所有回波数据，n为获取当前回波模式下第n类回波数据
    *(默认: 0)* 

#### 1.3 数据包分析
1. `enable_packet_loss_tool`: 启用基于序列号的数据包丢失统计  
   *(默认: false)*

2. `enable_packet_timeloss_tool`: 启用基于时间戳的数据包丢失检测  
   *(默认: false)*

3. `packet_timeloss_tool_continue`: 持续监控时间戳连续性
   *(默认: false)*

4. `update_function_safety_flag`: 解析功能安全字段，访问funcSafety获取解析结果，部分机械雷达支持
   *(默认: false)*

#### 1.4 校准设置
1. `distance_correction_lidar_flag`: 为机械式激光雷达启用光心修正
   *(Pandar/QT/OT/XT/JT系列可选)*

2. `xt_spot_correction`:  启用XT16/32 S形分层校正  
   *(默认: false)*

#### 1.5 Performance
1. `system_udpsocket_buffer_size`: UDP套接字缓冲区大小 (字节) 
    *(默认: 1500)*

#### 1.6 坐标变化相关
1. `transform_param`: 详见定义[TransformParam](../libhesai/Common/include/hs_com.h)，实现坐标转换

2. `remake_config`: 详见定义[RemakeConfig](../libhesai/Common/include/hs_com.h)，实现数据按照水平角和垂直角位置进行重排

#### 1.7 其他配置
1. `pcap_play_in_loop`: 允许循环播放PCAP数据
      *(默认: false)*

2. `thread_num`: 多线程解析，为1时为单线程解析，大于1时会开启多线程解析

3. `enable_udp_thread`: 是否允许接收来自雷达的点云数据
      *(默认: true)*

4. `enable_parser_thread`: 是否允许解析线程工作
      *(默认: true)*

5. `et_blooming_filter_flag`: 启用ET2.10高反展宽滤除功能
      *(默认: false)*

---

### 2 输入参数 [InputParam](../libhesai/driver_param.h)

#### 2.1 连接基础
1. `source_type`: 数据源选择  
   - `DATA_FROM_PCAP`: PCAP文件回放
   - `DATA_FROM_LIDAR`: 网络连接
   - `DATA_FROM_SERIAL`: 串口连接

2. `ptc_mode`: 通信协议模式  
   - `0`: TCP (默认)
   - `1`: SSL加密

#### 2.2 网络配置
1. `device_ip_address`: 激光雷达IP地址
   *(默认: 192.168.1.201)*

2. `udp_port`: 点云数据端口  
   *(默认: 2368)*

3. `ptc_port`: 控制协议端口  
   *(默认: 9347)*

4. `multicast_ip_address`: 组播地址 
   *(空字符串表示不设置组播地址)*

5. `fault_message_port`: 故障报文目的端口号，与udp_port不同时配置

6. `host_ip_address`: 主机IP地址

7. `device_udp_src_port`: 点云源端口号，通过点云源IP地址+源端口号筛选点云，有效值[1024， 65535]

8. `device_fault_port`: 故障报文源端口号，通过故障报文源IP地址+源端口号筛选故障报文，有效值[1024， 65535]

9. `use_ptc_connected`: 是否使用PTC通讯

10. `host_ptc_port`: PTC通信时指定源端口号，配置为0时，系统随机分配源端口号

#### 2.3 文件路径
1. `pcap_path`: PCAP文件路径  
   *(离线回放PCAP必需)*

2. `correction_file_path`: 校准文件路径
   *(默认需要CSV格式，部分雷达支持DAT格式)*

3. `firetimes_path`: 发光时刻修正文件路径

#### 2.4 串口配置 (仅JT16)
1. `rs485_com`: 点云接受串口  
    *(示例：COM3 或 /dev/ttyUSB0)*

2. `rs232_com`: 命令发送串口

3. `point_cloud_baudrate`: RS485波特率  
    *(默认: 3125000)*

4. `rs485_baudrate`: OTA命令波特率
    *(默认: 115200)*

5. `rs232_baudrate`: 正常命令波特率  
    *(默认: 9600)*

6. `correction_save_path`: 串口获取角度校准文件后的本地保存路径

#### 2.5 安全配置
1. `certFile`: 客户端证书路径  

2. `privateKeyFile`: 私钥路径 

3. `caFile`: CA证书路径 

#### 2.6 雷达配置相关
1. `standby_mode`: PTC连接成功后，配置雷达工作模式

2. `speed`: PTC连接成功后，配置雷达转速
#### 2.7 配置超时信息
1. `recv_point_cloud_timeout`: 初始化中接收点云数据的超时时间(s)，大于等于0时有效

2. `ptc_connect_timeout`: 初始化时PTC连接的超时时间(s)，大于等于0时有效

---

### 3 驱动参数 [DriverParam](../libhesai/driver_param.h)

#### 3.1 日志配置
1. `log_level`: 日志详细级别  
   - `0`: 调试
   - `1`: 信息 (默认)
   - `2`: 警告
   - `3`: 错误

2. `log_Target`: 输出目标  
   - `0`: 运行终端 (默认)
   - `1`: 文件
   - `2`: 同时输出

3. `log_path`: 日志文件目录  
   *(默认: ./logs)*

4. `use_gpu`: 是否使用gpu解析