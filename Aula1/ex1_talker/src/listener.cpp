#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class SubscriberNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stringSubscriber_;
    std::string parameter;

    void the_funcion_to_subscribe(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),"%s", msg->data.c_str());
        RCLCPP_INFO(get_logger(), "the world");
    }
    
public:
SubscriberNode(const std::string& name) : Node(name) {

    stringSubscriber_ = this->create_subscription<std_msgs::msg::String>("um_otimo_topico",10,
                                                    std::bind(&SubscriberNode::the_funcion_to_subscribe, 
                                                    this, _1)); //because the function has one arg
    //RCLCPP_INFO(get_logger(), "Subscribber created");
    }
    ~SubscriberNode();
};


SubscriberNode::~SubscriberNode()
{
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<SubscriberNode>("um_exemplar_ouvinte"); //'node' is pointer to node (smart pointer and shared)
    
    rclcpp::spin(node);
    //exit
    rclcpp::shutdown();
    return 0;
}
