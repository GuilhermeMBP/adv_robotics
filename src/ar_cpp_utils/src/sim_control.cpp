/*
Copyright (c) 2021, Hugo Costelha
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the Player Project nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


This applications controls the simulation for the second laboratory work, part
of the Advanced Robotics class in the Electrical Engineering Masters at the 
Polytechnic of Leiria, Portugal.
It simulates the timings and responses regarding picking up and leaving the parts,
taking into account the robot position. It also simulates the robot charging and
discharging.
*/

// ROS API
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp> // Pose2D
#include <nav_msgs/msg/odometry.hpp> // Odometry
#include <sensor_msgs/msg/battery_state.hpp> // BatteryState
#include <tf2/utils.h> // Geometry transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
// Our files
#include "ar_cpp_utils/LocalFrameWorldFrameTransformations.hpp"
#include "ar_utils/srv/start_charging.hpp"

// Other includes
#include <mutex>
#include <atomic>
#include <math.h>

#define RAD2DEG(x) x/3.1415927*180
// Beware that these two numberss cannot be changed without changing the code
//below to account for the new values.
#define NUM_PARTS 5 // Number of available parts/input/outputs
#define NUM_PMAN 3 // Number of processing machines
#define NUM_DUNITS 2 // Number of delivery units
#define NUM_CUNITS 3 // Number of charging units

enum part_state_t {
  PART_UNPROCESSED = 0, // Part was not yet processed
  PART_IN_PROCESS, // Part is being processed
  PART_PROCESSED, // Part was already processed
  PART_DELIVERED, // Part was already processed and delivered at its destination
};

enum forklift_state_t {
  FORKLIFT_UNKNOWN = 0,
  FORKLIFT_UP = 1,
  FORKLIFT_DOWN = 2,
  FORKLIFT_MOVING_UP = 3,
  FORKLIFT_MOVING_DOWN = 4
};


class Rect
{
public:
  Rect(point_2d top_left, point_2d bottom_right) : tl(top_left), br(bottom_right) {}
  
  point_2d tl; // Top left corner
  point_2d br; // Bottom right corner
};

class SimControl : public rclcpp::Node
{
  public:
    SimControl() : Node("sim_control")
    {
      /// ROS variables/objects
      std::string robot_name = "/robot_0/";

      // Forklift related
      forklift_state_ = FORKLIFT_DOWN;

      // Battery information
      battery_state_.voltage = 12.0;
      battery_state_.temperature = NAN;  // Not measured
      battery_state_.current = NAN;
      battery_state_.charge = NAN;
      battery_state_.capacity = NAN;
      battery_state_.design_capacity = NAN;
      battery_state_.percentage = 0.80;
      battery_state_.power_supply_status = 
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      battery_state_.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      battery_state_.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      battery_state_.present = true;
      battery_state_.cell_voltage = {NAN, NAN, NAN};
      battery_state_.cell_temperature = {NAN, NAN, NAN};
      battery_state_.location = "MAIN_SLOT";
      battery_state_.serial_number = "SIMULATED_BATTERY";

      // Parts related
      parts_status_msg_.data.clear();
      for(uint i=0; i < NUM_PARTS; i++)
        parts_status_msg_.data.push_back(PART_UNPROCESSED);

      pub_parts_status_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
            "parts_sensor", 
            rclcpp::QoS(1).reliable().transient_local());
      // Publisher for the battery level
      pub_batttery_state_ = create_publisher<sensor_msgs::msg::BatteryState>(
            "battery/state", rclcpp::QoS(rclcpp::KeepLast(1)));
      // Publisher for the velocity commands
      pub_vel_ = create_publisher<geometry_msgs::msg::Twist>(
        robot_name + "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)));

      /// Setup subscribers
      // Real, error-free robot and parts pose
      rclcpp::QoS qos = rclcpp::QoS(5);
      sub_real_pose_.subscribe(this, robot_name + "base_pose_ground_truth", qos.get_rmw_qos_profile());
      for (int i = 0; i < NUM_PARTS; i++)
        sub_parts_location_[i].subscribe(this, "/robot_" + std::to_string(i+1) + "/base_pose_ground_truth", qos.get_rmw_qos_profile());

      // build policy type: ApproximateTime<Odometry, Odometry, ...>
      sync_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
          nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry,
          nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>>>(
        message_filters::sync_policies::ApproximateTime<
          nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry,
          nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>(10),
      sub_real_pose_, sub_parts_location_[0], sub_parts_location_[1],
      sub_parts_location_[2], sub_parts_location_[3], sub_parts_location_[4]);

      sync_->setAgePenalty(0.15);
      sync_->registerCallback(std::bind(&SimControl::posesCallback, this,
         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
         std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

      // Setup subscriber for the forlift commands
      sub_forklift_ = create_subscription<std_msgs::msg::Int8>(
        robot_name + "cmd_forklift", 1,
          std::bind(&SimControl::forkliftCallback, this,
                    std::placeholders::_1));

      // Advertise the battery charging service
      svc_charge_ = create_service<ar_utils::srv::StartCharging>(
          robot_name + "battery/charge",
          std::bind(&SimControl::startChargeRequest,
                    this, 
                    std::placeholders::_1,
                    std::placeholders::_2));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Simulation control started...");

      // Start the main loop using a timer at 10 times per second
      main_loop_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimControl::mainLoop, this));
    }

    /**
     * Service callback for charging requests
     */
    bool startChargeRequest(
      const std::shared_ptr<ar_utils::srv::StartCharging::Request> req,
      std::shared_ptr<ar_utils::srv::StartCharging::Response> res)
    {
      // Store a local copy of the robot pose and velocity
      pose_vel_mutex_.lock();
      geometry_msgs::msg::Pose2D robot_pose = global_robot_pose_;
      geometry_msgs::msg::Pose2D robot_speed = global_robot_speed_;
      pose_vel_mutex_.unlock();

      battery_state_mutex_.lock();
      // Are we cancelling the charge?
      if(req->charge == false)
      {
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        res->charging = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery not charging...");
        battery_state_mutex_.unlock();
        return true;
      }

      // If the robot is not stopped, or too far, return false response
      if((abs(sqrt(pow(robot_speed.x,2) + 
                   pow(robot_speed.y,2))) > MAX_LIN_SPEED_ ||
         (abs(robot_speed.theta) > MAX_ANG_SPEED_)) ||
         (is_robot_in_charging_unit(robot_pose.x, robot_pose.y) == -1))
      {
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        res->charging = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery not charging...");
      } else
      {
        // The robot as low speed and is in a charging unit. Lets recharge it!
        battery_state_.power_supply_status = 
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        res->charging = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery is charging...");
      }
      battery_state_mutex_.unlock();
      return true;
    }

    /**
     * Periodic callback to control charging/discharging
     */
    void batteryCallback()
    {
      battery_state_mutex_.lock();
      if(battery_state_.power_supply_status == 
         sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)
      {
        if(battery_state_.percentage > BATT_DISCHARGE_DELTA_)
          battery_state_.percentage -= BATT_DISCHARGE_DELTA_;
        else
          battery_state_.percentage = 0;
      } else
      {
        // Check if our position is still good, and that we are stopped
        pose_vel_mutex_.lock();
        geometry_msgs::msg::Pose2D robot_pose = global_robot_pose_;
        geometry_msgs::msg::Pose2D robot_speed = global_robot_speed_;
        pose_vel_mutex_.unlock();
        if((abs(sqrt(pow(robot_speed.x,2) + 
                     pow(robot_speed.y,2))) > MAX_LIN_SPEED_ ||
           (abs(robot_speed.theta) > MAX_ANG_SPEED_)) ||
           (is_robot_in_charging_unit(robot_pose.x, robot_pose.y) == -1))
        {
          battery_state_.power_supply_status = 
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        } else
        {
          if(battery_state_.percentage < 1.0 - BATT_CHARGE_DELTA_)
          {
            battery_state_.percentage += BATT_CHARGE_DELTA_;
          } else {
            battery_state_.percentage = 1.0;
            battery_state_.power_supply_status =
              sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
          }
        }
      }

      // Pubish the battery state
      pub_batttery_state_->publish(battery_state_);

      // If the battery level is too low, the robot should not move
      if(battery_state_.percentage < 0.03)
      {
        geometry_msgs::msg::Twist vel_cmd;
        vel_cmd.angular.z = 0.0;
        vel_cmd.linear.x = 0.0;
        pub_vel_->publish(vel_cmd);
      }
      battery_state_mutex_.unlock();
    }

    /**
     * Receive and store desired forklift value
     */
    void forkliftCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
      // Process forklift request (store desired position)
      if (msg->data == forklift_state_)
        return; // Nothing to do.
      if ((msg->data == 1) && (forklift_state_ != FORKLIFT_UP))
      {
        forklift_state_ = FORKLIFT_MOVING_UP; // Moving up
        last_forklift_cmd_time_ = get_clock()->now();
        return;
      }
      if ((msg->data == 2) && (forklift_state_ != FORKLIFT_DOWN))
      {
        forklift_state_ = FORKLIFT_MOVING_DOWN; // Moving down
        last_forklift_cmd_time_ = get_clock()->now();
        return;
      }
    }

    /**
     * Receive and store current robot real pose
     */
    void posesCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr& robot,
      const nav_msgs::msg::Odometry::ConstSharedPtr& part1,
      const nav_msgs::msg::Odometry::ConstSharedPtr& part2,
      const nav_msgs::msg::Odometry::ConstSharedPtr& part3,
      const nav_msgs::msg::Odometry::ConstSharedPtr& part4,
      const nav_msgs::msg::Odometry::ConstSharedPtr& part5)
    {
      // Call the batrery callback on every iteration
      this->batteryCallback();

      std::vector<nav_msgs::msg::Odometry::ConstSharedPtr> parts_msgs = 
        {part1, part2, part3, part4, part5};

      // Store real, error-free pose values given by the simulator
      geometry_msgs::msg::Pose2D robot_pose, robot_speed;
      robot_pose.x = robot->pose.pose.position.x;
      robot_pose.y = robot->pose.pose.position.y;
      robot_pose.theta = tf2::getYaw(robot->pose.pose.orientation);
      robot_speed.x = robot->twist.twist.linear.x;
      robot_speed.y = robot->twist.twist.linear.y;
      robot_speed.theta = robot->twist.twist.angular.z;

      // Update global values
      pose_vel_mutex_.lock();
      global_robot_pose_ = robot_pose;
      global_robot_speed_ = robot_speed;
      pose_vel_mutex_.unlock();

      // If enough time has elapsed, assume the forklift has finished moving
      if(((forklift_state_ == FORKLIFT_MOVING_UP) ||
          (forklift_state_ == FORKLIFT_MOVING_DOWN)) &&
         ((get_clock()->now() - last_forklift_cmd_time_).seconds() > FORKLIFT_TIME_))
      {
        if(forklift_state_ == FORKLIFT_MOVING_UP)
        {
          forklift_state_ = FORKLIFT_UP;
        } else if(forklift_state_ == FORKLIFT_MOVING_DOWN)
        {
          forklift_state_ = FORKLIFT_DOWN;
        }
      }

      // Change each part information according to the forklift position
      //and the robot pose
      bool part_ext_status_changed = false;
      int pu_num = -1;
      int du_num = -1;
      for(uint part_num=0; part_num < NUM_PARTS; part_num++)
      {
        // Get this part position
        point_2d part_pos;
        part_pos.x = parts_msgs[part_num]->pose.pose.position.x;
        part_pos.y = parts_msgs[part_num]->pose.pose.position.y;
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                    "box%d position: (%.2f, %.2f)",
                    part_num+1, part_pos.x, part_pos.y);

        switch(parts_state_[part_num])
        {
          // Part is still unprocessed
          case part_state_t::PART_UNPROCESSED:
            // Check if the part is in a processing unit and the robot is not
            // in that processing unit.
            pu_num = is_part_in_processing_unit(part_pos);
            if (pu_num != -1 )
            {
              if ((forklift_state_ == FORKLIFT_DOWN) &&
                  (!is_position_in_location(robot_pose.x, robot_pose.y,
                                           processing_units_location_[pu_num])))
              {
                // The robot has placed the part, it is now processing
                parts_state_[part_num] = part_state_t::PART_IN_PROCESS;
                parts_location_[part_num] = pu_num; // Processing unit number
                part_ext_status_changed = true;
                parts_status_msg_.data[part_num] = part_state_t::PART_IN_PROCESS;
                int32_t dt = lrint(20.0+(rand()*1.0/RAND_MAX)*30);
                parts_timestamps_[part_num].sec = robot->header.stamp.sec + dt;
                parts_timestamps_[part_num].nanosec = robot->header.stamp.nanosec;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d is being processed...",
                            part_num+1);
              }
            }
            break;

          // Part is being processed in a processing station
          case part_state_t::PART_IN_PROCESS:
            // If the robot is inside the processing unit, change the part back
            // into UNPROCESSED STATE.
            if (is_position_in_location(robot_pose.x, robot_pose.y,
                  processing_units_location_[parts_location_[part_num]]))
            {
              parts_state_[part_num] = part_state_t::PART_UNPROCESSED;
              part_ext_status_changed = true;
              parts_status_msg_.data[part_num] = part_state_t::PART_UNPROCESSED;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                          "Part %d is no longr being processed...",
                          part_num+1);
            } else
            {
              // If enough time passed by, part is done processing
              if((robot->header.stamp.sec >= parts_timestamps_[part_num].sec) &&
                (robot->header.stamp.nanosec >= parts_timestamps_[part_num].nanosec))
              {
                parts_state_[part_num] = part_state_t::PART_PROCESSED;
                part_ext_status_changed = true;
                parts_status_msg_.data[part_num] = part_state_t::PART_PROCESSED;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d has finished processing...",
                            part_num+1);
              }
            }
            break;

          // Part is done processing, we can now pick it up
          case part_state_t::PART_PROCESSED:
            // If the part is processed, it is on a delivery unit, and the robot
            // is not in that delivery unit, we can consider it delivered.
            du_num = is_part_in_delivery_unit(part_pos);
            
            if( du_num != -1 )
            {
              if ((forklift_state_ == FORKLIFT_DOWN) &&
                 (!is_position_in_location(robot_pose.x, robot_pose.y,
                delivery_units_location_[du_num])))
              {
                // The robot has placed the part, it is now delivered
                parts_state_[part_num] = part_state_t::PART_DELIVERED;
                part_ext_status_changed = true;
                parts_status_msg_.data[part_num] = part_state_t::PART_DELIVERED;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Part %d has been delivered...",
                            part_num+1);
              }
            }
            break;

          case part_state_t::PART_DELIVERED:
            // Nothing to do/check
            break;

          //default:
          //    ROS_ERROR("Unknown part state!!");
        }
      }

     if(part_ext_status_changed)
        pub_parts_status_->publish(parts_status_msg_);
    }

    void mainLoop()
    {
      // If we are shutting down, cancl the timer callback
      if(rclcpp::ok() == false)
      {
        main_loop_timer_->cancel();
        return;
      }

      if(first_time_)
      {
        pub_parts_status_->publish(parts_status_msg_);
        first_time_ = false;
      } else
        main_loop_timer_->cancel();
    }

  private:
    bool is_position_in_location(double x, double y, Rect location)
    {
      if((x > location.tl.x) && (x < location.br.x) &&
         (y < location.tl.y) && (y > location.br.y))
        return true;
      else
        return false;
    }

    int is_part_in_processing_unit(point_2d part_location)
    {
      for(uint pu_num=0; pu_num < NUM_PMAN; pu_num++)
      {
        if(is_position_in_location(part_location.x,
                                   part_location.y,
                                   processing_units_location_[pu_num]))
          return pu_num;
      }
      return -1;
    }

    int is_part_in_delivery_unit(point_2d part_location)
    {
      for(uint du_num=0; du_num < NUM_DUNITS; du_num++)
      {
        if(is_position_in_location(part_location.x,
                                   part_location.y,
                                   delivery_units_location_[du_num]))
          return du_num;
      }
      return -1;
    }

    int is_robot_in_charging_unit(double x, double y)
    {
      for(uint cu_num=0; cu_num < NUM_CUNITS; cu_num++)
      {
        if(is_position_in_location(x, y, charging_units_location_[cu_num]))
          return cu_num;
      }
      return -1;
    }

    bool first_time_ = true;
    // Timer for the periodic callback (main loop)
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    // Forklift related variables
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_forklift_;
    // Forklift information
    const float FORKLIFT_TIME_ = 0.5;  // Down position
    // 0 - unknown, 1 - up, 2 - down, 3 - moving up, 4 - moving down
    forklift_state_t forklift_state_;
    rclcpp::Time last_forklift_cmd_time_;

    // Battery simulation related variables
    sensor_msgs::msg::BatteryState battery_state_;
    std::mutex battery_state_mutex_;
    rclcpp::TimerBase::SharedPtr battery_timer_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr 
        pub_batttery_state_;
    rclcpp::Service<ar_utils::srv::StartCharging>::SharedPtr svc_charge_;
    // For now we assume a simple model, with a constant battery discharge and
    // charge 
    const float BATT_DISCHARGE_DELTA_ = 0.0005;  // Per 0.1 sec 
    const float BATT_CHARGE_DELTA_ = 0.010;  // Per iteration

    // Robot position and velocity, used to control the charging too
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_real_pose_;
    geometry_msgs::msg::Pose2D global_robot_pose_, global_robot_speed_;
    std::mutex pose_vel_mutex_;
    // Parts location
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_parts_location_[NUM_PARTS];
    std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>>> sync_;

    // Maximum error to consider in desired orientation
    const float MAX_X_OFFSET_ = 0.35; // [m]
    const float MAX_DIST_ERROR_ = 0.10; // [m]
    const float MAX_ANG_ERROR_ = 0.14; // [rad] (8º)
    // Maxumum speed to grab/drop the parts
    const float MAX_LIN_SPEED_ = 0.01; // [m/s]
    const float MAX_ANG_SPEED_ = 0.017; // [rad/s] (1º/s)

    // Store the positions of the parts
    const float DA_ = 0.0; // Distance to part in [m]
    const point_2d  input_parts_[NUM_PARTS] =
      {{-4.90, -7.00+DA_}, // 1 (left)
       {-2.45, -7.00+DA_}, // 2
       {0.00, -7.00+DA_}, // 3
       {2.45, -7.00+DA_}, // 4
       {4.90, -7.00+DA_}};// 5 (right)
    const point_2d process_parts_[NUM_PMAN] =
      {{-2.45, 2.00-DA_}, // Left 1
       {0.00, 2.00-DA_}, // Center 2
       {2.45, 2.00-DA_}};// Right 8
    const point_2d output_parts_[NUM_PARTS] =
      {{-6.50, 7.40-DA_}, // 1
       {-3.00, 7.40-DA_}, // 2
       {0.50, 7.40-DA_}, // 3
       {4.30, 7.40-DA_}, // 4
       {7.00, 7.40-DA_}};// 5

    // Store the location of the processing units
    const Rect processing_units_location_[NUM_PMAN] =
      {Rect({-3.70, 2.45}, {-1.30, 0.50}), // Left processing unit
       Rect({-1.30, 2.45}, {1.20, 0.50}),  // Center processing unit
       Rect({1.20, 2.45}, {3.50, 0.50})};  // Right processing unit

    // Store the location of the delivery units
   const Rect delivery_units_location_[NUM_DUNITS] =
      {Rect({-5.90, 2.45}, {-3.75, 0.50}), // Left delivery unit
       Rect({3.60, 2.45}, {6.00, 0.50})};  // Right delivery unit
    /*const Rect delivery_units_location_[NUM_DUNITS] =
      {Rect({-8.00, 8.00}, {-4.90, 5.00}), // Left delivery unit
       Rect({-4.90, 8.00}, {-1.20, 5.00}), // Center left delivery unit
       Rect({-1.20, 8.00}, {2.50, 5.00}),  // Center delivery unit
       Rect({2.50, 8.00}, {6.15, 5.00}),   // Center right delivery unit
       Rect({6.15, 8.00}, {8.00, 5.00})};  // Right delivery unit
    */
    // Store the location of the chrging units
    const Rect charging_units_location_[NUM_CUNITS] =
      {Rect({-5.05, -2.05}, {-4.55, -2.55}), // Left charging unit
       Rect({-0.25, -2.55}, {0.25, -3.05}),  // Center charging unit
       Rect({4.55, -1.75}, {5.05, -2.25})};  // Right charging unit

    // Parts status
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      pub_parts_status_;
    std_msgs::msg::UInt8MultiArray parts_status_msg_;
    part_state_t parts_state_[NUM_PARTS] = {PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED,
                                            PART_UNPROCESSED};

    // Parts location
    // 0:4 - Input wharehouse n
    // 5 - On the move
    // 6:13 - In processing machine n-5
    // 14:18 - Input wharehouse n-13
    uint parts_location_[NUM_PARTS] = {0, 1, 2, 3, 4};
    // Parts processing timestamp
    builtin_interfaces::msg::Time parts_timestamps_[NUM_PARTS];
};


/**
  * Main function for controlling the simulation.
  */
int main(int argc, char** argv)
{
  // Init ROS
  rclcpp::init(argc, argv);
  // Infinite loop with one instance of our class
  rclcpp::spin(std::make_shared<SimControl>());
  rclcpp::shutdown();
  return 0;
}
