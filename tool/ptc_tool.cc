/**
 * @file ptc_tool.cc
 * @author your name (you@domain.com)
 * @brief This file gives an example of modifying radar parameters using the PTC Directive.
 * @version 0.1
 * @date 2023-12-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hesai_lidar_sdk.hpp"

int main(int argc, char* argv[], char* envp[])
{
#ifndef _MSC_VER
  system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"");
#endif
   HesaiLidarSdk<LidarPointXYZIRT> sample;
  DriverParam param;
  // set connect param
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";
  param.input_param.device_ip_address = "192.168.1.201";
  param.input_param.ptc_port = 9347;
  param.input_param.udp_port = 2368;
  param.input_param.host_ip_address = "192.168.1.100";
  param.input_param.multicast_ip_address = "";

  // connect lidar
  sample.Init(param);

  std::string information = R"(
//*************************************** HESAI TECH *****************************************//
//******************************* Information 0f Configuration *******************************//
                              )";
  std::cout << information << std::endl;

  // set param
  // Some lidar sets IP and then restarts. If you have a register setting requirement, it will invalidate your register setting.
  // If you don't have the need to set the register to lose the set value when the power goes down, just put the lidar network setting after all the settings.
  int IsSetLidarNet = 0;
  if (IsSetLidarNet) {
    std::string net_IP = "192.168.1.202";
    std::string net_mask = "255.255.255.0";
    std::string net_getway = "192.168.1.1";
    uint8_t vlan_flag = 0;
    uint16_t vlan_ID = 0;
    if (sample.lidar_ptr_->ptc_client_->SetNet(net_IP, net_mask, net_getway, vlan_flag, vlan_ID)) {
      std::cout << "setNetforOT faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetNet successed!" << std::endl;
      printf("Current Lidar IP: %s, Current Lidar Mask: %s, Current Lidar GetWay: %s\n",net_IP.c_str(), net_mask.c_str(), net_getway.c_str());
      printf("Current Vlan_flag: %d, Current Vlan_ID: %d\n", vlan_flag, vlan_ID);
    }
  } else {
    std::string destination_ipv4 = "192.168.1.100";
    uint16_t UDP_Port = 2368;
    uint16_t GPS_UDP_port = 10110;

    uint8_t return_mode = 2;

    uint8_t enable_sync_angle = 1;
    uint16_t sync_angle = 20;
    
    uint32_t standby_mode = 0;
    uint32_t speed = 600;

    // uint32_t TmbFPGARegister1 = 0x90000004;
    // uint32_t TmbFPGARegister2 = 0x8001f138;
    // uint32_t TmbFPGARegister1_data = 0x01;
    // uint32_t TmbFPGARegister2_data = 0x202;
    // uint32_t FPGARegister = 0x43c081f0;
    // uint32_t FPGARegister_data = 0x04;

    // set return_mode
    if (sample.lidar_ptr_->ptc_client_->SetReturnMode(return_mode)) {
      std::cout << "SetReturnMode faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetReturnMode successed!" << std::endl;
      printf("Current Return_Mode: %d\n", return_mode);
    }

    // set syncangle
    if (sample.lidar_ptr_->ptc_client_->SetSyncAngle(enable_sync_angle, sync_angle)) {
      std::cout << "SetSyncAngle faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetSyncAngle successed!" << std::endl;
      printf("Current Enable_Sync_Angle: %d, Current Sync_Angle: %d\n", enable_sync_angle, sync_angle);
    }

    // set destination ip and port
    if (sample.lidar_ptr_->ptc_client_->SetDesIpandPort(destination_ipv4, UDP_Port, GPS_UDP_port)) {
      std::cout << "SetDesIpandPort faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetDesIpandPort successed!" << std::endl;
      printf("Current Destination_ipv4: %s, Current UDP_Port: %d, Current GPS_UDP_Port: %d\n", destination_ipv4.c_str(), UDP_Port, GPS_UDP_port);
    }

    // Set standby_mode
    if (sample.lidar_ptr_->ptc_client_->SetStandbyMode(standby_mode)) {
      std::cout << "SetStandbyMode faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetStandbyMode successed!" << std::endl;
      printf("Current StandbyMode: %d\n", standby_mode);
    }

    // set spin_speed
    if (sample.lidar_ptr_->ptc_client_->SetSpinSpeed(standby_mode)) {
      std::cout << "SetSpinSpeed faild! Please make sure your input is valid." << std::endl;
    } else {
      std::cout << "SetSpinSpeed successed!" << std::endl;
      printf("Current speed: %d\n", speed);
    }

    // // SetTmbFPGARegister
    // if (sample.lidar_ptr_->ptc_client_->SetTmbFPGARegister(TmbFPGARegister1, TmbFPGARegister1_data) || sample.lidar_ptr_->ptc_client_->SetTmbFPGARegister(TmbFPGARegister2, TmbFPGARegister2_data)) {
    //   std::cout << "SetTmbRegister faild! Please make sure your input is valid." << std::endl;
    // } else {
    //   std::cout << "SetTmbRegister Successed!" << std::endl;
    //   printf("Address 0x%08x : Data: 0x%04x\n", TmbFPGARegister1, TmbFPGARegister1_data);
    //   printf("Address 0x%08x : Data: 0x%04x\n", TmbFPGARegister2, TmbFPGARegister2_data);
    // }

    // // SetFPGARegister
    // if (sample.lidar_ptr_->ptc_client_->SetFPGARegister(FPGARegister, FPGARegister_data)) {
    //   std::cout << "SetFPGARegister faild! Please make sure your input is valid." << std::endl;
    // } else {
    //   std::cout << "SetFPGARegister Successed!" << std::endl;
    //     printf("Address 0x%08x : Data: 0x%04x\n", FPGARegister, FPGARegister_data);
    // }
  }
  std::string information2 = R"(
//****************************** The configuration of is complete! ***************************//
//*************************************** HESAI TECH *****************************************//
  )";
  std::cout << information2 << std::endl;


  return 0;
}