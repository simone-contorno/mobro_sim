#include <mobro_sim/robot.h>

/* Unicycle class */
class Unicycle : public Robot
{
public:
  /* Constructor */
  explicit Unicycle(rclcpp::Node::SharedPtr node, const Traj &traj, std::string name) : Robot(node, traj, name) {}
};
