# 丢包率统计

## 1 概述
丢包率统计功能实现了针对禾赛激光雷达UDP数据的实时丢包统计，基于每个UDP点云数据包中的序列号（UDP Sequence） 字段进行丢包检测和统计

## 2 丢包率统计的实现方式
统计SDK解析的激光雷达UDP数据的丢包率一共有三种方式

### 方法1：统计一段时间内的丢包率和时间戳跳变率
在tool文件夹的[packet_loss_tool.cc](..\tool\packet_loss_tool.cc)中param部分，设置好网络配置（默认的雷达设置状态下无需更改）
```cpp
param.input_param.device_ip_address = "192.168.1.201";  //雷达ip
param.input_param.ptc_port = 9347;  //无需更改             
param.input_param.udp_port = 2368;  //udp端口
param.input_param.host_ip_address = "192.168.1.100";  //上位机ip
```
可以修改统计时间
```cpp
float run_time = 15;  //统计15s
```
#### 编译
```bash
# 0. 安装依赖项
sudo apt update && sudo apt install -y libpcl-dev libpcap-dev libyaml-cpp-dev

# 1. 导航到源目录
cd HesaiLidar_SDK_2.0/tool

# 2. 创建build目录并导航到build目录
mkdir -p build && cd build

# 3. 使用Cmake配置项目
cmake ..

# 4. 编译SDK
make
```
#### 运行
```bash
./packet_loss_tool
```
#### 输出示例：
```log
total recevice packet time: 15000ms  //统计时间
total receviced packet count: 93229  //统计总包数
package loss: 
total loss packet count: 0  //统计总丢包数  
timestamp loss: 
total loss packet count: 0  //统计总时间戳跳变数
```

### 方法2：统计1s内两次丢包之间总共的丢包率
在[test.cc](..\test\test.cc)中param部分，设置param.decoder_param.enable_packet_loss_tool = true，开启丢包统计功能
编译和运行方式参考方法1
当出现丢包时，终端会打印类似如下警告信息：
```log
[WARNING] pkt loss freq: 3 / 56268
```
表示在统计周期1s内，从上一次丢包开始，到下一次新的丢包之间，共丢失了3个包，应收到包的总数为56268

### 方法3：用户自行增加功能统计丢包率
根据用户自己需求，可增加回调函数，用于在终端输出当前帧丢包率和累计丢包率，例如在[test.cc](..\test\test.cc)中增加如下回调函数：
```cpp
void packetLossCallback(const uint32_t& total_packets, const uint32_t& lost_packets) {
  static uint32_t last_total = 0;
  static uint32_t last_loss = 0;

  // Compute packet count and loss since last frame
  uint32_t frame_total = total_packets - last_total;
  uint32_t frame_loss  = lost_packets - last_loss;

  // Update static tracking variables
  last_total = total_packets;
  last_loss  = lost_packets;

  // Calculate loss rates
  double frame_loss_rate = frame_total == 0 ? 0.0 :
                           static_cast<double>(frame_loss) / frame_total * 100.0;
  double total_loss_rate = total_packets == 0 ? 0.0 :
                           static_cast<double>(lost_packets) / total_packets * 100.0;

  // Print results
  printf("[Frame Loss Rate]  %.2f%% (%u / %u)\n", frame_loss_rate, frame_loss, frame_total);
  printf("[Total Loss Rate]  %.2f%% (%u / %u)\n", total_loss_rate, lost_packets, total_packets);
}
```
并调用该回调函数：
```cpp
sample.RegRecvCallback(packetLossCallback);
```
编译和运行方式参考方法1
#### 输出示例:
```log
[Frame Loss Rate]  97.47% (20241 / 20767)
[Total Loss Rate]  2.12% (20241 / 956569)
```
