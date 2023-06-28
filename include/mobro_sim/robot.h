#ifndef ROBOT_H
#define ROBOT_H

#include <mobro_sim/traj.h>
#include <mobro_sim/utils.h>

/* Dynamic gains */
struct Gains
{
  double d;   // carrot distance
  double dt;  // trajectory angle step size 
};

/* Goal parameters */
struct Goal
{
  Vector2d pd;	// position
  Vector2d vd;	// velocity
  Vector2d ad;	// accleration
  bool use_traj;
};

/* Robot class */
class Robot
{        
protected:
    rclcpp::Node* node;
    const Traj & traj;
    std::string name;
    
    /* Carrot */
    JointState carrot;
    rclcpp::Publisher<JointState>::SharedPtr carrot_pub;

    /* Goal */
	
    // Future goals
    PubFloat32MultiArray pub_goals;
    std_msgs::msg::Float32MultiArray goals;
    size_t vars = 3;
    size_t dim = 100*vars;

	// Manual
    std_msgs::msg::Bool manual_goal_flag;
	geometry_msgs::msg::PoseStamped manual_goal;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_goal_pub;
	bool manual_goal_used = false;
	
	// Automatic
	geometry_msgs::msg::PoseStamped goal;
    rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub;
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub;
    
	// Twist
	Twist cmd_vel;
    geometry_msgs::msg::Twist des_twist;
    rclcpp::Publisher<Twist>::SharedPtr des_twist_pub;

    /* Joints */
	rclcpp::Subscription<JointState>::SharedPtr js_sub;
    rclcpp::Publisher<Float32MultiArray>::SharedPtr cmd_pub;

	/* Parameters */
    double theta = 0.;

	/* Functions */
    Goal currentGoal(const rclcpp::Time &now, double& traj_vel);

	/* Understand */
    inline Vector2d linearVelocity(bool relative = true) const
    {
        if(relative)
        return {cmd_vel.linear.x, cmd_vel.linear.y};
        const auto c{cos(theta)};
        const auto s{sin(theta)};
        return {c*cmd_vel.linear.x + s*cmd_vel.linear.y, s*cmd_vel.linear.x - c*cmd_vel.linear.y};
    }

	/* Parameters Callback handling */
	rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_change;
    void declareParams();
    inline SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
    inline double declareParam(std::string name, double default_value, double lower, double upper);

public:
	/* Constructor*/
    explicit Robot(rclcpp::Node::SharedPtr node, const Traj &traj, std::string name);
    
	/* Goal publisher */
	void publish(const rclcpp::Time &now, double& traj_vel);

	/* Read model name */
    static std::string read_name(rclcpp::Node::SharedPtr node);

	/* Get */
    inline std::chrono::milliseconds samplingTime() const {return std::chrono::milliseconds{(int)(1000*dt)};}
    std::string getName() {return this->name;}

    Gains gains;
    double angle = -M_PI/2; // trajectory starting angle
    double dt = 0.1;        // step size
};

#endif
