#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ExNode : public rclcpp::Node{
public:
    ExNode(const std::string & node_name) : Node(node_name){
        this->declare_parameter<std::string>("param","Around");
        timer_ = this->create_wall_timer(std::chrono::seconds(3), // period
                                        std::bind(&ExNode::the_funcion_to_publish_cb, this)); //bind function to obj
        stringPublisher_ = this->create_publisher<std_msgs::msg::String>("um_otimo_topico", 10);
    }
    
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stringPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    void the_funcion_to_publish_cb(){
        auto msg = std_msgs::msg::String();
        msg.data = this->get_parameter("param").as_string();
        stringPublisher_->publish(msg);
    }
};

int main(int argc, char **argv){
    // initialization
    rclcpp::init(argc,argv);

    //code
    // creating shared pointer to node
    auto node = std::make_shared<ExNode>("um_belo_no"); //'node' is pointer to node (smart pointer and shared)
    rclcpp::spin(node);
    //exit
    rclcpp::shutdown();
    return 0;
}