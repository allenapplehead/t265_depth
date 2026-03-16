#include <rclcpp/rclcpp.hpp>
#include <t265_depth/t265_depth.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<t265_depth::t265Depth>());
    rclcpp::shutdown();
    return 0;
}
