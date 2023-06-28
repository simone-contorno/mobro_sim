#include <mobro_sim/robot.h>

class Bicycle : public Robot
{  
    double L{1.6};
    double beta_max{1.5};
    double beta;
    std::string steering_joint;
    Float32MultiArray cmd;

public:
	explicit Bicycle(rclcpp::Node::SharedPtr node, const Traj &traj, std::string name, double L=1.6, const std::string &joint = "frame_to_handlebar") : Robot(node, traj, name), L{L}, steering_joint{joint}
	{
		js_sub = node->create_subscription<JointState>("robot/joint_states", 2, [&](const JointState::SharedPtr msg)
		{
			size_t i{};
			for(const auto &name: msg->name)
			{
				if(name == steering_joint) beta = msg->position[i];
				i++;
			}
		});

		cmd.data.resize(2);
		cmd_pub = node->create_publisher<Float32MultiArray>("robot/cmd", 1);
	}
};
