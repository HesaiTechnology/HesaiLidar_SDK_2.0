# 发送PTC指令 

## 1 概述
本功能实现了通过SDK向雷达发送PTC指令，获取雷达的信息，以及设置雷达参数

## 2 发送PTC指令的方式
### 编译
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
### 运行
```bash
./packet_loss_tool
```
运行ptc.tool.cc
