// "Copyright [year] <Copyright Owner>"

#include "ignition_platform.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ignition_platform::IgnitionPlatform>();
  node->preset_loop_frequency(60);
  as2::spinLoop(node);  
  
  rclcpp::shutdown();
  return 0;
}
