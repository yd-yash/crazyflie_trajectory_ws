/*
 * cf_motor_plugin.cpp
 * Gazebo Classic model plugin for Crazyflie 2.1.
 * Applies thrust and torques directly in the physics update loop.
 * Reference: rotors_simulator (ethz-asl/rotors_simulator)
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{

class CrazyflieMotorPlugin : public ModelPlugin
{
public:
  CrazyflieMotorPlugin() : ModelPlugin(), u1_(0), u2_(0), u3_(0), u4_(0) {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    link_ = model->GetLink("cf_base_link");
    if (!link_) {
      gzerr << "[CrazyflieMotorPlugin] Link 'cf_base_link' not found!\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);

    ctrl_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cf1/control_debug",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
          std::lock_guard<std::mutex> lock(mutex_);
          u1_ = msg->data[0];
          u2_ = msg->data[1];
          u3_ = msg->data[2];
          u4_ = msg->data[3];
        }
      }
    );

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieMotorPlugin::OnUpdate, this));

    gzmsg << "[CrazyflieMotorPlugin] Loaded. Subscribed to /cf1/control_debug\n";
  }

  void OnUpdate()
  {
    double u1, u2, u3, u4;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      u1 = u1_; u2 = u2_; u3 = u3_; u4 = u4_;
    }

    // Get current body orientation
    ignition::math::Quaterniond q = link_->WorldPose().Rot();

    // Rotate thrust from body Z to world frame
    ignition::math::Vector3d thrust_body(0.0, 0.0, u1);
    ignition::math::Vector3d thrust_world = q * thrust_body;

    // Apply force in world frame
    link_->AddForce(thrust_world);

    // Apply torques in body frame
    link_->AddRelativeTorque(ignition::math::Vector3d(u2, u3, u4));
  }

private:
  physics::ModelPtr   model_;
  physics::WorldPtr   world_;
  physics::LinkPtr    link_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ctrl_sub_;
  std::mutex mutex_;
  double u1_, u2_, u3_, u4_;
};

GZ_REGISTER_MODEL_PLUGIN(CrazyflieMotorPlugin)

} // namespace gazebo

