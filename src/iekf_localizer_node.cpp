#include "iekf_localizer/iekf_localizer.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iekf_localizer::IEKFLocalizer>());
  rclcpp::shutdown();

  return 0;
}
