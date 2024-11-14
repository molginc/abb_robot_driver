/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <cstdlib>
#include <stdexcept>

#include <ros/ros.h>

#include <controller_manager/controller_manager.h>

#include <abb_robot_msgs/GetRobotControllerDescription.h>

#include <abb_egm_hardware_interface/egm_hardware_interface.h>

#include <abb_robot_cpp_utilities/parameters.h>
#include <abb_egm_hardware_interface/egmfeedback.h>
#include <abb_egm_hardware_interface/feedback.h>

namespace
{
/**
 * \brief Name for ROS logging in the 'main' context.
 */
constexpr char ROS_LOG_MAIN[]{"main"};

/**
 * \brief Timeout [s] for waiting on ROS services.
 */
constexpr double ROS_SERVICE_TIMEOUT{30.0};
}

/**
 * \brief creates a ros service to advertise feedback 
 * \return true if service call was succesful
 */
std::tuple<std::vector<std::string>,std::vector<std::string>> FormatFeedback(std::map<std::string, std::vector<double>> feedback);

/**
 * \brief Gets a ABB robot controller description via a ROS service call.
 *
 * Note: This function is a workaround for the fact that the description is collected
 *       during runtime (in another node), and is not know at launch time.
 *
 * \param nh specifying a node handle to use for reading and setting ROS parameters.
 *
 * \throw std::runtime_error if the description could not be gotten.
 */
void getRobotControllerDescription(ros::NodeHandle& nh);

int main(int argc, char** argv)
{
  //--------------------------------------------------------
  // Preparations
  //--------------------------------------------------------
  ros::init(argc, argv, "egm_hardware_interface");

  // Create a node handle, in the namespace where the
  // hardware interface's ROS interfaces should operate in.
  //
  // Note: Using "egm" to indicate that the ROS interfaces
  //       are related to Externally Guided Motion (EGM)
  //       communication.
  ros::NodeHandle nh{"egm"};

  // Create a node handle, in the namespace where the
  // hardware interface should read parameters from.
  ros::NodeHandle nh_hw{"~"};

  //--------------------------------------------------------
  // Start node execution
  //--------------------------------------------------------
  try
  {
    // Process ROS interfaces in the background.
    ros::AsyncSpinner spinner{4};
    spinner.start();

    // Get a description of the targeted ABB robot controller.
    getRobotControllerDescription(nh_hw);

    // Create and initialize the hardware interface.
    abb::robot::EGMHardwareInterface hw{};
    hw.init(nh, nh_hw);

    // Create a controller manager, to manage the
    // hardware interface and active controllers.
    controller_manager::ControllerManager cm{&hw, nh};
    
    ros::Publisher egm_feedback = nh.advertise<abb_egm_hardware_interface::feedback>("egm_feedback", 1000);
    abb_egm_hardware_interface::feedback msg;

    // Run the control loop.
    ros::Time time_now{};
    ros::Duration period{1.0/250.0};
    while(ros::ok())
    {
      period = hw.waitForMessage();
      time_now = ros::Time::now();
      hw.read(time_now, period);

      // get feedback from read service and format as vector of string messages for broadcasting 
      auto [keys,vals]= FormatFeedback(hw.getFeedback());

      // ::abb_egm_hardware_interface::feedback test_vec;
      // publish feedback
      msg.keys = keys;
      msg.values = vals;
      egm_feedback.publish(msg);
      ROS_INFO_STREAM_NAMED(ROS_LOG_MAIN, "Feedback keys: " << boost::algorithm::join(keys, ", "));
      ROS_INFO_STREAM_NAMED(ROS_LOG_MAIN, "Feedback values: " << boost::algorithm::join(vals, ", "));

      cm.update(time_now, period, hw.resetNeeded());
      hw.write(time_now, period);
    }
  }
  catch(const std::runtime_error& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Runtime error: '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(const std::invalid_argument& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Invalid argument: '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(const std::out_of_range& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Out of range: '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(const std::exception& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Exception '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(...)
  {
    ROS_FATAL_NAMED(ROS_LOG_MAIN, "Unknown exception");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}


std::tuple<std::vector<std::string>,std::vector<std::string>> FormatFeedback(std::map<std::string, std::vector<double>> feedback)
{
  std::vector<std::string> keys;
  std::vector<std::string> vals;

  for (auto const& [key, val] : feedback)
  {
    keys.push_back(key);
    std::string str_val = "";
    for (auto const& v : val)
    {
      str_val += std::to_string(v) + " ";
    }
    vals.push_back(str_val);
  }
  std::tuple feedback_map = std::make_tuple(keys,vals);
  // std::tuple<std::vector<std::string>,std::vector<std::string>> feedback_map;
  return feedback_map;
}


void getRobotControllerDescription(ros::NodeHandle& nh)
{
  // Get indicator for allowing no ROS service timeout during initialization.
  bool no_service_timeout{false};
  abb::robot::utilities::getParameter(nh, "no_service_timeout", no_service_timeout, false);

  // Components for a ROS service to get a description
  // of the targeted robot controller.
  //
  // Note: The description is assumed to have been
  //       collected via RWS, and parsed, in another ROS
  //       node, which provides a ROS service to share
  //       the description.
  std::string service_name{"rws/get_robot_controller_description"};
  abb_robot_msgs::GetRobotControllerDescription service_result{};

  ROS_DEBUG_STREAM_NAMED(ROS_LOG_MAIN, "Wait for ROS service '" << service_name << "'");

  // Wait for the ROS service.
  if(no_service_timeout)
  {
    ros::service::waitForService(service_name);
  }
  else
  {
    if(!ros::service::waitForService(service_name, ros::Duration(ROS_SERVICE_TIMEOUT)))
    {
      throw std::runtime_error{"Timed out while waiting for ROS service '" + service_name + "'"};
    }
  }

  // Call the ROS service.
  if(!ros::service::call(service_name, service_result))
  {
    throw std::runtime_error{"Failed to call ROS service '" + service_name + "'"};
  }

  // Set the robot controller description parameter for
  // the hardware interface's initialization.
  //
  // Note: This deviates from best practices and
  //       is only used as a workaround because
  //       the robot controller description is
  //       not know at launch time.
  nh.setParam("robot_controller_description", service_result.response.description);
}
