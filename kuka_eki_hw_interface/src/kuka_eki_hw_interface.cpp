/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, 3M
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the copyright holder, nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Brett Hemes (3M) <brhemes@mmm.com>


#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <angles/angles.h>

#include <tinyxml.h>

#include <kuka_eki_hw_interface/kuka_eki_hw_interface.h>
//Data logging
#include <iostream>
#include <fstream>


namespace kuka_eki_hw_interface
{

KukaEkiHardwareInterface::KukaEkiHardwareInterface() : joint_position_(n_dof_, 0.0), joint_velocity_(n_dof_, 0.0),
    joint_effort_(n_dof_, 0.0), joint_position_command_(n_dof_, 0.0), joint_names_(n_dof_), deadline_(ios_),
    eki_server_socket_(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
{
}


KukaEkiHardwareInterface::~KukaEkiHardwareInterface() {
  /*//LOGGING
  if (fich_logging_.is_open()){
    fich_logging_.close();
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Fichero de logging cerrado");
  }*/
}


void KukaEkiHardwareInterface::eki_check_read_state_deadline()
{
  // Check if deadline has already passed
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    eki_server_socket_.cancel();
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Sleep until deadline exceeded
  deadline_.async_wait(boost::bind(&KukaEkiHardwareInterface::eki_check_read_state_deadline, this));
}


void KukaEkiHardwareInterface::eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                                  boost::system::error_code* out_ec, size_t* out_length)
{
  *out_ec = ec;
  *out_length = length;
}


bool KukaEkiHardwareInterface::eki_read_state(std::vector<double> &joint_position,
                                              std::vector<double> &joint_velocity,
                                              std::vector<double> &joint_effort,
                                              int &cmd_buff_len)
{

  //logging corrientes
  std::vector<double> joint_current(n_dof_, 0.0);

  static boost::array<char, 2048> in_buffer;

  // Read socket buffer (with timeout)
  // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
  deadline_.expires_from_now(boost::posix_time::seconds(eki_read_state_timeout_));  // set deadline
  boost::system::error_code ec = boost::asio::error::would_block;
  size_t len = 0;
  eki_server_socket_.async_receive(boost::asio::buffer(in_buffer),
                                   boost::bind(&KukaEkiHardwareInterface::eki_handle_receive, _1, _2, &ec, &len));
  do
    ios_.run_one();
  while (ec == boost::asio::error::would_block);
  if (ec)
    return false;

  // Update joint positions from XML packet (if received)
  if (len == 0)
    return false;

  // Parse XML
  TiXmlDocument xml_in;
  in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
  xml_in.Parse(in_buffer.data());

  
  TiXmlPrinter printer;
  printer.SetIndent( "    " );
  xml_in.Accept( &printer );
  std::string xmltext = printer.CStr();
  ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "El buffer de entrada es: ");
  ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", xmltext);
  





  TiXmlElement* robot_state = xml_in.FirstChildElement("RobotState");
  if (!robot_state)
    return false;
  TiXmlElement* pos = robot_state->FirstChildElement("Pos");
  TiXmlElement* vel = robot_state->FirstChildElement("Vel");
  TiXmlElement* eff = robot_state->FirstChildElement("Eff");
  TiXmlElement* curr = robot_state->FirstChildElement("Curr");//Corrientes de ejes  
  TiXmlElement* robot_command = robot_state->FirstChildElement("RobotCommand");
  if (!pos || !vel || !eff || !robot_command)
    return false;

  // Extract axis positions
  double joint_pos;  // [deg]
  double joint_vel;  // [%max]
  double joint_eff;  // [Nm]
  double joint_curr; //corrientes
  //char axis_name[] = "A1";  //ÑAPA TOTAL DE ROS
  

  std::string ValoresEjes = "\n\nLos valores de los ejes son: \n";
  for (int i = 0; i < n_dof_; ++i)
  {
    /*pos->Attribute(axis_name, &joint_pos);
    joint_position[i] = angles::from_degrees(joint_pos);  // convert deg to rad
    vel->Attribute(axis_name, &joint_vel);
    joint_velocity[i] = joint_vel;
    eff->Attribute(axis_name, &joint_eff);
    joint_effort[i] = joint_eff;*/
    //axis_name[1]++; //ÑAPA TOTAL DE ROS

    pos->Attribute(NombreEjesKuka_[i], &joint_pos);
    joint_position[i] = angles::from_degrees(joint_pos);  // convert deg to rad
    vel->Attribute(NombreEjesKuka_[i], &joint_vel);
    joint_velocity[i] = joint_vel;
    eff->Attribute(NombreEjesKuka_[i], &joint_eff);
    joint_effort[i] = joint_eff;

    //Corrientes
    curr->Attribute(NombreEjesKuka_[i], &joint_curr);
    joint_current[i] = joint_curr;

    //logging
    fich_logging_ << i << "\t" << joint_pos << "\t" << joint_vel << "\t" << joint_eff << "\t" << joint_curr << "\n";
    fich_logging_.flush();
    /*ValoresEjes += "    ";
    ValoresEjes += NombreEjesKuka_[i];
    ValoresEjes += " ";
    ValoresEjes += std::to_string(joint_position[i]);*/
  }
  //ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", ValoresEjes);



  // Extract number of command elements buffered on robot
  robot_command->Attribute("Size", &cmd_buff_len);

  return true;
}


bool KukaEkiHardwareInterface::eki_write_command(const std::vector<double> &joint_position_command)
{
  TiXmlDocument xml_out;
  TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
  TiXmlElement* pos = new TiXmlElement("Pos");
  TiXmlText* empty_text = new TiXmlText("");
  robot_command->LinkEndChild(pos);
  pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
  //char axis_name[] = "A1";//ÑAPA ROS
  //ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Mandando valores para\n");
  for (int i = 0; i < n_dof_; ++i)
  {
    //ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", NombreEjesKuka_[i] << ": " << joint_position_command[i]);//DEBUG
    pos->SetAttribute(NombreEjesKuka_[i], std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
    //axis_name[1]++; //ÑAPA ROS
  }
  xml_out.LinkEndChild(robot_command);

  TiXmlPrinter xml_printer;
  xml_printer.SetStreamPrinting();  // no linebreaks
  xml_out.Accept(&xml_printer);

  TiXmlPrinter printerDebug;
  printerDebug.SetIndent( "    " );
  xml_out.Accept( &printerDebug );
  std::string xmltext = printerDebug.CStr();
  /*ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "El buffer de salida  es: ");
  ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", xmltext);*/
  

  size_t len = eki_server_socket_.send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                          eki_server_endpoint_);

  return true;
}


void KukaEkiHardwareInterface::init()
{
  //LOGGING
  std::time_t t = std::time(0);
  std::tm* now = std::localtime(&t);  
  std::string fichstr = "/home/benat/Downloads/borrar/logging_eki_" + std::to_string(now->tm_year + 1900) + '-' + std::to_string(now->tm_mon + 1) + '-' + std::to_string(now->tm_mday) + '-' + std::to_string(now->tm_hour) + '-' + std::to_string(now->tm_min) + '-' + std::to_string(now->  tm_sec) + ".tsv";
  fich_logging_.open(fichstr, std::fstream::in | std::fstream::out | std::fstream::trunc);
  if (fich_logging_.is_open())
  {
    fich_logging_ << "logging EKI\neje\tpos\tvel\ttor\tcurr\n";
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Se ha escrito en el fichero");
    fich_logging_.flush();//fich_logging_.close();
  }
  else
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "NO se ha escrito en el fichero");



  // Get controller joint names from parameter server
  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'controller_joint_names' on the parameter server.");
  }

  // Get EKI parameters from parameter server
  const std::string param_addr = "eki/robot_address";
  const std::string param_port = "eki/robot_port";
  const std::string param_socket_timeout = "eki/socket_timeout";
  const std::string param_max_cmd_buf_len = "eki/max_cmd_buf_len";

  if (nh_.getParam(param_addr, eki_server_address_) &&
      nh_.getParam(param_port, eki_server_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Configuring Kuka EKI hardware interface on: "
                          << eki_server_address_ << ", " << eki_server_port_);
  }
  else
  {
    std::string msg = "Failed to get EKI address/port from parameter server (looking for '" + param_addr +
                      "', '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }

  if (nh_.getParam(param_socket_timeout, eki_read_state_timeout_))
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Configuring Kuka EKI hardware interface socket timeout to "
                          << eki_read_state_timeout_ << " seconds");
  }
  else
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Failed to get EKI socket timeout from parameter server (looking "
                          "for '" + param_socket_timeout + "'), defaulting to " +
                          std::to_string(eki_read_state_timeout_)  + " seconds");
  }

  if (nh_.getParam(param_max_cmd_buf_len, eki_max_cmd_buff_len_))
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Configuring Kuka EKI hardware interface maximum command buffer "
                          "length to " << eki_max_cmd_buff_len_);
  }
  else
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Failed to get EKI hardware interface maximum command buffer length "
                          "from parameter server (looking for '" + param_max_cmd_buf_len + "'), defaulting to " +
                          std::to_string(eki_max_cmd_buff_len_));
  }

  // Create ros_control interfaces (joint state and position joint for all dof's)

  n_dof_= 8;
  ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "El numero de grados de libertad es: " << n_dof_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Joint state interface
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("kukw_eki_hw_interface", "Loaded Kuka EKI hardware interface");
}


void KukaEkiHardwareInterface::start()
{
  ROS_INFO_NAMED("kuka_eki_hw_interface", "Starting Kuka EKI hardware interface...");

  // Start client
  ROS_INFO_NAMED("kuka_eki_hw_interface", "... connecting to robot's EKI server...");
  boost::asio::ip::udp::resolver resolver(ios_);
  eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});
  boost::array<char, 1> ini_buf = { 0 };
  eki_server_socket_.send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);  // initiate contact to start server

  // Start persistent actor to check for eki_read_state timeouts
  deadline_.expires_at(boost::posix_time::pos_infin);  // do nothing unit a read is invoked (deadline_ = +inf)
  eki_check_read_state_deadline();

  // Initialize joint_position_command_ from initial robot state (avoid bad (null) commands before controllers come up)
  if (!eki_read_state(joint_position_, joint_velocity_, joint_effort_, eki_cmd_buff_len_))
  {
    std::string msg = "Failed to read from robot EKI server within alloted time of "
                      + std::to_string(eki_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
                      "on the robot controller and all configurations are correct.";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  joint_position_command_ = joint_position_;

  ROS_INFO_NAMED("kuka_eki_hw_interface", "... done. EKI hardware interface started!");
}


void KukaEkiHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
  if (!eki_read_state(joint_position_, joint_velocity_, joint_effort_, eki_cmd_buff_len_))
  {
    std::string msg = "Failed to read from robot EKI server within alloted time of "
                      + std::to_string(eki_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
                      "on the robot controller and all configurations are correct.";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
}


void KukaEkiHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
  // only write if max will not be exceeded
  if (eki_cmd_buff_len_ < eki_max_cmd_buff_len_)
    eki_write_command(joint_position_command_);

  // underflow/overflow checking
  // NOTE: this is commented as it results in a lot of logging output and the use of ROS_*
  //       logging macros breaks incurs (quite) some overhead. Uncomment and rebuild this
  //       if you'd like to use this anyway.
  //if (eki_cmd_buff_len_ >= eki_max_cmd_buff_len_)
  //  ROS_WARN_STREAM("eki_hw_iface RobotCommand buffer overflow (curent size " << eki_cmd_buff_len_
  //                  << " greater than or equal max allowed " << eki_max_cmd_buff_len_ << ")");
  //else if (eki_cmd_buff_len_ == 0)
  //  ROS_WARN_STREAM("eki_hw_iface RobotCommand buffer empty");
}

} // namespace kuka_eki_hw_interface
