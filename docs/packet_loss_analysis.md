# Packet Loss Analysis
The function of packet loss analysis realize real-time packet loss statistics for UDP data of Hesai lidars, which bases on UDP sequence of UDP packets. 


## Preparation
There are three ways to calculate the packet loss rate of lidar UDP data parsed by SDK. The following is **Method 1**. For other methods, please refer to **References**.

#### Method 1: Calculate the packet loss rate and the rate of timestamp during a certain period of time
Open [packet_loss_tool.cc](../tool/packet_loss_tool.cc)
```cpp
// Network Configuration（no need to change if parameters of lidar is default）
param.input_param.device_ip_address = "192.168.1.201";  // lidar IP
param.input_param.ptc_port = 9347;  // TCP port
param.input_param.udp_port = 2368;  // UDP port
param.input_param.host_ip_address = "192.168.1.100";  // Host PC IP address
```

The time of calculating packet loss can be adjusted as needed.
```cpp
float run_time = 15;  // calculate 15s
```


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
Once compiled successfully, go to the build folder, execute the generated packet_loss_tool executable.
```bash
./packet_loss_tool
```
Output example：
```log
total recevice packet time: 15000ms  // calculate total time
total receviced packet count: 93229  // calculate total packet
package loss: 
total loss packet count: 0  // calculate the total number of packet loss  
timestamp loss: 
total loss packet count: 0  // calculate the total time of packet loss 
```


## Reference
#### Method 2: Calculate the total packet loss rate between two consecutive packet losses within 1 second
Open [test.cc](../test/test.cc)
```cpp
// Enable packet loss calculate
param.decoder_param.enable_packet_loss_tool = true; // enable the function of calculating packet loss
```
The compilation and run methods refer to Method 1. When packet loss occurs, the terminal will print a warning message similar to the following:
```log
[WARNING] pkt loss freq: 3 / 56268
```
It indicates that within the 1-second statistical period, starting from the last packet loss and up to the next new packet loss, a total of 3 packets were lost, and the total number of packets that should have been received was 56268.

#### Method 3：Users can increase the function of calculating packet loss rate by themselves
According to the user's own requirements, a callback function can be added to output the current frame packet loss rate and the cumulative packet loss rate on the terminal, such as adding a callback function in [test.cc](../test/test.cc):
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
And call this callback function：
```cpp
sample.RegRecvCallback(packetLossCallback);
```
The compilation and run methods refer to Method 1.

Output example:
```log
[Frame Loss Rate]  97.47% (20241 / 20767)
[Total Loss Rate]  2.12% (20241 / 956569)
```
