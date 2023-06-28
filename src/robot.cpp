#include <mobro_sim/utils.h>
#include <mobro_sim/robot.h>

/*!
 * Compute and publish the current goal.
 * @param now clock time
 * @param traj_vel trajectory scalar coefficients
 */
Goal Robot::currentGoal(const rclcpp::Time &now, double& traj_vel)
{
    /* Take current trajectory */
    auto [pd,vd,ad] = traj.currentTraj(angle, gains.dt); {}
    
    /* Update trajectory parameters */
    traj_vel = gains.dt;  // update trajectory velocity
    
    /* Update goal */
    goal.header.stamp = now;
    goal.header.frame_id = "map";
    goal.pose.position.x = pd[0];
    goal.pose.position.y = pd[1];
    const auto theta_d{atan2(vd[1], vd[0])};
    goal.pose.orientation.w = cos(theta_d/2);
    goal.pose.orientation.z = sin(theta_d/2);
    des_twist.linear.x = vd[0];
    des_twist.linear.y = vd[1];
    des_twist_pub->publish(des_twist);

    static double t0{-1};

    if(!manual_goal_used)
    {
        t0 = -1;
        goal_pub->publish(goal);
        return {pd,vd,ad,true};
    }

    if(linearVelocity().norm() < 1e-3)
    {
        if(t0 < 0) t0 = now.seconds();
        if((now.seconds()-t0) > 5)
        {
            manual_goal_used = false;
            goal_pub->publish(goal);
            return {pd,vd,ad,true};
        }
    }
    pd[0] = manual_goal.pose.position.x;
    pd[1] = manual_goal.pose.position.y;
    vd.setZero();

    return {pd,vd,ad,false};
}

/*!
 * Initialize the robot object (constructor). 
 * @param node node pointer.
 * @param traj trajectory object.
 * @param name robot name.
 */
Robot::Robot(rclcpp::Node::SharedPtr node, const Traj &traj, std::string name) :
    node{node.get()},
    traj{traj},
    name{name}
{
    /* Joints */
    carrot_pub = node->create_publisher<JointState>(name + "/joint_states", 1);

    // Carrot
    carrot.name = {"carrot", "carrot_disable"};
    carrot.position = {0, 0};

    /* Goal */
    goal_sub = node->create_subscription<PoseStamped>("/move_base_simple/goal", 1, [&](PoseStamped::UniquePtr goal)
    {
        manual_goal = *goal;
        manual_goal_used = true;
        goal_pub->publish(*goal);
    });
    goal_pub = node->create_publisher<PoseStamped>(name + "/goal", 1);
    manual_goal_pub = node->create_publisher<std_msgs::msg::Bool>(name + "/manual_goal", 1);
    des_twist_pub = node->create_publisher<Twist>(name + "/des_twist", 1);

    /* Parameters */
    declareParams();
    param_change = node->add_on_set_parameters_callback(std::bind(&Robot::onParameterChange, this, std::placeholders::_1));
    
    /* Future goals */
    pub_goals = node->create_publisher<std_msgs::msg::Float32MultiArray>(name + "/goals", 1);
    goals.data.resize(dim);
}

/*!
 * Publish carrot position.
 * @param now current clock time.
 * @param traj_vel current trajectory velocity.
 */
void Robot::publish(const rclcpp::Time &now, double& traj_vel)
{
    /* Current goal */
    auto [pd_,vd_,ad_,flag] = currentGoal(now, traj_vel); {}

    /* Future goals */
    double angle_temp = angle;
    for (size_t i = 0; i < dim; i += vars)
    {
      auto [pd,vd,ad] = traj.currentTraj(angle_temp, gains.dt); {}
      auto theta_d{atan2(vd[1], vd[0])};
      goals.data[i] = pd[0];
      goals.data[i+1] = pd[1];
      goals.data[i+2] = theta_d;
    }

    pub_goals->publish(goals);

    /* Manual goal 
    manual_goal_flag.data = flag;
    manual_goal_pub->publish(manual_goal_flag);
    */

    /* Carrot */
    carrot.header.stamp = now;
    carrot.position[0] = gains.d;
    carrot.position[1] = 0;
    carrot_pub->publish(carrot);
}

/*!
 * Read the model name.
 * @param node node pointer.
 */
std::string Robot::read_name(rclcpp::Node::SharedPtr node)
{
  const auto rsp_param{std::make_shared<rclcpp::SyncParametersClient>
                      (node, "/robot/robot_state_publisher")};
  std::cout << "Reading robot description... " << std::flush;
  while(true)
  {
    rclcpp::spin_some(node);
    rsp_param->wait_for_service();
    const auto urdf_xml{rsp_param->get_parameter<std::string>("robot_description")};
    if(urdf_xml.empty())
      continue;

    auto model{std::make_unique<urdf::Model>()};
    model->initString(urdf_xml);
    std::cout << " found " << model->getName() << std::endl;
    return model->getName();
  }
}

/*!
 * Define the server parameters.
 */
void Robot::declareParams()
{
    gains.d = declareParam(name + ".carrot.d", .1, -1., 1.);
    gains.dt = declareParam(name + ".traj.dt", .1, .01, 1.);
}

/*!
 * Declare a server parameter.
 * @param name parameter name.
 * @param default_value starting value.
 * @param lower minimum value.
 * @param upper maximum value.
 */
double Robot::declareParam(std::string name,
                           double default_value,
                           double lower, 
                           double upper)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  const auto step{0.001};
  descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                     .set__from_value(lower)
                                     .set__to_value(upper)
                                     .set__step(step)};
  default_value = lower + step*std::round((default_value-lower)/step);
  return node->declare_parameter<double>(name, default_value, descriptor);
}

/*!
 * Check if a server parameter changes at run time and update it.
 * @param parameters curret declared parameters.
 */
SetParametersResult Robot::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  for(auto &param: parameters)
  {
    auto &nm{param.get_name()};
    const auto val{param.as_double()};
    if(nm == name + ".carrot.d") gains.d = val;
    else if(nm == name + ".traj.dt") 
    {
      gains.dt = val;
      std::cout << "[SIMULATOR - " << name << "] " << "Trajectory velocity updated." << std::endl;
    }
  }
  return SetParametersResult().set__successful(true);
}
