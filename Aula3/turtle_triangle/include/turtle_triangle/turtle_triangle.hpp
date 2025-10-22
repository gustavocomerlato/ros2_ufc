#ifndef TURTLE_TRIANGLE__TURTLE_TRIANGLE_HPP_
#define TURTLE_TRIANGLE__TURTLE_TRIANGLE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>


namespace turtle_triangle
{
    enum class turtle_state{
        FORWARD,
        STOP_FORWARD,
        TURN,
        STOP_TURN
    };

    using namespace std::chrono_literals;
    class TrianglePathPublisher : public rclcpp::Node{
        public:
            TrianglePathPublisher() : Node("turtle_triangle")
            {
                // parâmetros configuráveis do triângulo
                this->declare_parameter<double>("initial_velocity", 1.0);
                this->declare_parameter<double>("speedup", 0.2);
                this->declare_parameter<std::vector<double>>("side_lengths", {2.0, 2.0, 2.0});
                this->declare_parameter<std::vector<double>>("angles_deg", {120.0, 120.0, 120.0});
                this->declare_parameter<int>("pub_period_ms", 5); //periodo de taxa de publicacao

                //populando os campos dos parâmetros
                initial_velocity_ = this->get_parameter("initial_velocity").as_double();
                speedup_          = this->get_parameter("speedup").as_double();
                side_lengths_     = this->get_parameter("side_lengths").as_double_array();
                angles_deg_       = this->get_parameter("angles_deg").as_double_array();
                pub_period_ms_    = this->get_parameter("pub_period_ms").as_int();

                // setting up timer, velocity command publisher, subscription
                vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
                pose_sub_    = this->create_subscription<turtlesim::msg::Pose>(
                    "turtle1/pose", 5,
                    std::bind(&TrianglePathPublisher::poseCallback, this, std::placeholders::_1)
                );

                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(pub_period_ms_),
                    std::bind(&TrianglePathPublisher::timer_cb, this)
                );
            }

        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_; //publicador de comando de velocidade
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_; //inscrição na pose da tartaruga
            rclcpp::TimerBase::SharedPtr timer_;

            void timer_cb(); //função de callback do timer
            void poseCallback(const turtlesim::msg::Pose::SharedPtr); // função de inscrição de pose
            void publish_path(double linear, double angular); // função de publicação do caminho

            //parâmetros
            double initial_velocity_;
            double speedup_;
            std::vector<double> side_lengths_;
            std::vector<double> angles_deg_;
            int pub_period_ms_;
            double xy_tolerance_ = 0.02;
            double angle_tolerance_ = 0.01;

            //comandos de movimento da tartaruga
            void moveForward();
            void stopForward();
            void spinTurtle();
            void stopSpin();

            //funcoes auxiliares
            bool isCloseToGoal();
            bool isStopped();
            void printGoal();
            
            //estados da tartaruguinha
            turtle_state state_ = turtle_state::FORWARD;
            turtle_state last_state_ = turtle_state::FORWARD;
            bool first_goal_set = false;
            int num_laps_=0;
            int current_side_=0;
            turtlesim::msg::Pose::SharedPtr pose_; //pose atual da tartaruga
            turtlesim::msg::Pose goal_; //objetivo

    };

}

#endif // TURTLE_TRIANGLE__TURTLE_TRIANGLE_HPP_