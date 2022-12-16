//
// Created by arslan on 22.05.2022.
//
#include "ClapB7Driver.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClapB7Driver>());
    rclcpp::shutdown();
    return 0;
}