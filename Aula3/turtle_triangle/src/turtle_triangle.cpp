#include "turtle_triangle/turtle_triangle.hpp"

namespace turtle_triangle{

  //callbacks
  void TrianglePathPublisher::publish_path(double linear, double angular){
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
    vel_cmd_pub_->publish(vel_msg);
  }

  void TrianglePathPublisher::poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
  {
    pose_ = pose;
  }

  void TrianglePathPublisher::printGoal()
  {
    RCLCPP_INFO(this->get_logger(), "New goal: x=%.2f, y=%.2f, theta=%.2f",
                goal_.x, goal_.y, goal_.theta);
  }

  bool TrianglePathPublisher::isCloseToGoal(){
    // RCLCPP_INFO(this->get_logger(), "Distance to goal: x=%.2f, y=%.2f, theta=%.2f",
    //             std::fabs(pose_->x - goal_.x), std::fabs(pose_->y - goal_.y), std::fabs(pose_->theta - goal_.theta));
    if (std::fabs(pose_->x - goal_.x) < xy_tolerance_ &&
       std::fabs(pose_->y - goal_.y) < xy_tolerance_ &&
       std::fabs(pose_->theta - goal_.theta) < angle_tolerance_) {
          return true;
        }else{
          return false;
        }
  }

  bool TrianglePathPublisher::isStopped(){
    if (pose_->linear_velocity < 0.01*xy_tolerance_ &&
       pose_->angular_velocity < 0.01*angle_tolerance_) {
          return true;
        }else{
          return false;
        }
  }

  void TrianglePathPublisher::moveForward(){
    if (isCloseToGoal())
    {
      RCLCPP_INFO(this->get_logger(), "Turtle reached goal.");
      RCLCPP_INFO(this->get_logger(), "New State: STOP_FORWARD");
      last_state_ = state_;
      state_ = turtle_state::STOP_FORWARD;
      double linear = 0.0;
      double angular = 0.0;
      publish_path(linear,angular);
    }else{
      double linear = initial_velocity_ + num_laps_ * speedup_;
      double angular = 0.0;
      publish_path(linear, angular);
    }
  }

  void TrianglePathPublisher::stopForward(){
    if(isStopped()){
      RCLCPP_INFO(this->get_logger(), "Turtle stopped.");
      RCLCPP_INFO(this->get_logger(), "New State: TURN");
      last_state_ = state_;
      state_ = turtle_state::TURN;
      goal_.x = pose_->x; //mantem o objetivo em x e y
      goal_.y = pose_->y;
      goal_.theta = std::fmod(pose_->theta + angles_deg_[current_side_] * (M_PI / 180),
                              2*M_PI); //atualiza objetivo angular - "desenrola" o ângulo para [-pi, pi)
      if (goal_.theta > M_PI) goal_.theta -= 2 * M_PI;
      printGoal();
    }
  else
    {
      publish_path(0, 0);
    }
  }

  void TrianglePathPublisher::spinTurtle(){
    if (isCloseToGoal())
    {
      RCLCPP_INFO(this->get_logger(), "Turtle stopped.");
      RCLCPP_INFO(this->get_logger(), "New State: STOP_TURN");
      last_state_ = state_;
      state_ = turtle_state::STOP_TURN;
      double linear = 0.0;
      double angular = 0.0;
      publish_path(linear,angular);
    }else{
      double angular = initial_velocity_ + num_laps_ * speedup_;
      double linear = 0.0;
      publish_path(linear,angular);
    };
  }

  void TrianglePathPublisher::stopSpin(){
    if(isStopped()){
      RCLCPP_INFO(this->get_logger(), "Turtle stopped spinning.");
      RCLCPP_INFO(this->get_logger(), "New State: FORWARD");
      last_state_ = state_;
      state_ = turtle_state::FORWARD;
      goal_.x = cos(pose_->theta) * side_lengths_[current_side_] + pose_->x;
      goal_.y = sin(pose_->theta) * side_lengths_[current_side_] + pose_->y;
      goal_.theta = pose_->theta; 
      printGoal();
      current_side_++; // incrementa o lado atual sendo navegado
    }
  else
    {
      publish_path(0, 0);
    }
  }


  void TrianglePathPublisher::timer_cb(){
    //Primeira coisa: checa se já tem a pose no cache  
    if (!pose_)
    {
      return;
    }
    //checa se houve atualizações nos parâmetros
    initial_velocity_ = this->get_parameter("initial_velocity").as_double();
    speedup_          = this->get_parameter("speedup").as_double();
    side_lengths_     = this->get_parameter("side_lengths").as_double_array();
    angles_deg_       = this->get_parameter("angles_deg").as_double_array();
    pub_period_ms_    = this->get_parameter("pub_period_ms").as_int();
    if (!first_goal_set)
    {
      first_goal_set = true;
      state_ = turtle_state::FORWARD;
      goal_.x = cos(pose_->theta) * side_lengths_[0] + pose_->x;
      goal_.y = sin(pose_->theta) * side_lengths_[0] + pose_->y;
      goal_.theta = pose_->theta;
      printGoal();
      current_side_++;
    }

    //Checa o número de vezes que a tartaruga deu uma volta em um vértice
    if (current_side_ >= 3)
    {
      current_side_ = 0;
      num_laps_++;
      RCLCPP_INFO(this->get_logger(), "Turtle completed lap number %d!", num_laps_);
    }

    //Em cada tick do timer, checa o estado atual da tartaruga
    switch(this->state_){
      case turtle_state::FORWARD:
        moveForward();
        break;
      case turtle_state::STOP_FORWARD:
        stopForward();
        break;
      case turtle_state::TURN:
        spinTurtle();
        break;
      case turtle_state::STOP_TURN:
        stopSpin();
        break;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<turtle_triangle::TrianglePathPublisher>());
  rclcpp::shutdown();
  return 0;
}
