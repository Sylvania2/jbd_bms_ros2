/*

Copyright 2019 I-Quotient Robotics Co., Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "jbd_bms_status.h"

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("jbd_bms_status_node");
  //iqr::JbdBmsStatus jbd_bms_status(private_nh);

  iqr::JbdBmsStatus jbd_bms_status("jbd_bms_status");
  
  jbd_bms_status.initPort();

  uint16_t buffer_sum = 0x00;
  std::vector<uint8_t> buffer_v;
  rclcpp::Rate loop_rate(jbd_bms_status.looprate_);
  while (rclcpp::ok()) {
    jbd_bms_status.buffer_all_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_status_,
        jbd_bms_status.cmd_status_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.buffer_vol_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_voltage_,
        jbd_bms_status.cmd_voltage_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.dataParsing(jbd_bms_status.buffer_all_, jbd_bms_status.buffer_vol_);
    
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

#if 0
  ros::init(argc, argv, "jbd_bms_status_node");
  ros::NodeHandle private_nh("~");

  iqr::JbdBmsStatus jbd_bms_status(private_nh);  
  jbd_bms_status.initPort();

  uint16_t buffer_sum = 0x00;
  std::vector<uint8_t> buffer_v;
  ros::Rate loop_rate(jbd_bms_status.looprate_);
  while(ros::ok) {
    jbd_bms_status.buffer_all_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_status_,
        jbd_bms_status.cmd_status_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.buffer_vol_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_voltage_,
        jbd_bms_status.cmd_voltage_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.dataParsing(jbd_bms_status.buffer_all_, jbd_bms_status.buffer_vol_);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
#endif

