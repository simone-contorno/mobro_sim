#include <mobro_sim/utils.h>
#include <mobro_sim/robot.h>
#include <mobro_sim/bicycle.h>
#include <mobro_sim/unicycle.h>

int main(int argc, char** argv)
{
    /********************************************************************************/
    /********************* Trajectory and robots initialization *********************/
    /********************************************************************************/

    rclcpp::init(argc, argv);
    auto node{std::make_shared<rclcpp::Node>("simulate")};
    Traj traj;
    Traj traj_obs1;
    Traj traj_obs2;

    /* Check robot's name */

    std::string name = "";
    if (argc > 1) name = static_cast<string>(argv[1]);
    else 
    {
        std::cerr << "[ERROR] No name found." << std::endl;
        exit(0);
    }

    if (name != "r2d2" && name != "bike" && name != "zoe" && name !=  "rosbot")
    {
        std::cerr << "[ERROR] Model has not been implemented." << std::endl;
        exit(0);
    }
    
    //const auto name{Robot::read_name(node)};

    auto robot = std::make_unique<Robot>(node, traj, "robot"); 
    auto obs1 = std::make_unique<Robot>(node, traj_obs1, "obs1");  
    auto obs2 = std::make_unique<Robot>(node, traj_obs2, "obs2");  

    robot->gains.dt = 0.15;
    robot->angle = M_PI;
    obs1->gains.dt = 0.1; 
    obs2->gains.dt = 0.1; 

    std::cout << "[SIMULATOR] ";
    
    if (argc > 2)
    {   
        /********************************************************************************/
        /********************************** Simulation **********************************/
        /********************************************************************************/

        /* Simulation: circular path with 2 obstacles on circular paths */
        if (static_cast<string>(argv[2]) == "circle") 
        {
            std::cout << "Simulation: circular path with 2 dynamic obstacles on circular paths set." << std::endl;
            traj.path_num = 1;
            
            traj_obs1.path_num = 1;
            traj_obs2.path_num = 1;
            
            // Obstacle 1
            traj_obs1.r /= 6;      
            traj_obs1.b = traj.r + traj_obs1.r; 
            traj_obs1.a += 7;  
            traj_obs1.w *= 3;

            // Obstacle 2
            traj_obs2.r /= 6;      
            traj_obs2.b = traj.r - traj_obs2.r; 
            traj_obs2.a -= 7;  
            traj_obs2.w *= 3;
        }

        /********************************************************************************/
        /************************************ ROSbot ************************************/
        /********************************************************************************/

        else if (name == "rosbot")
        {
            // Robot
            traj.path_num = 3;
            traj.coeff_x = 4.;
            traj.line_offset_x = 2.;
            robot->angle = M_PI;
            robot->gains.dt = 0.15;
            robot->dt = 0.2;
            
            // Obstacle
            traj_obs1.path_num = 4;
            traj_obs1.line_offset_x = 2.;
            obs1->gains.dt = 0.15; 

            /* ROSbot: horizontal line path with single fixed obstacle on horizontal line path */
            if (static_cast<string>(argv[2]) == "fix") 
            {
                std::cout << "ROSbot: horizontal line path with single fixed obstacle on horizontal line path set." << std::endl;
                traj_obs1.coeff_y = 0.; 
            }

            /* ROSbot: horizontal line path with single dynamic obstacle on vertical line path */
            else if (static_cast<string>(argv[2]) == "dyn") 
            {
                std::cout << "ROSbot: horizontal line path with single dynamic obstacle on vertical line path set." << std::endl;
                traj_obs1.coeff_y = 2.;                
            }
        }

        /* Path does not exist. */
        else 
        {
            std::cout << "The chosen path does not exist." << std::endl;
            return 0;
        }
    }

    /* No path. */
    else 
    {
        std::cout << "No path chosen." << std::endl;
        return 0;
    }

    if (name == "bike" || name == "zoe")
    {
        // Rob
        double coeff = 1.5;
        traj.line_offset_x *= coeff; 
        traj.coeff_x *= coeff;
        traj.coeff_y *= coeff;
        robot->gains.dt /= coeff;
        
        // Obstacle1
        obs1->gains.dt /= coeff;
        traj_obs1.line_offset_x = traj.line_offset_x; 

        // Obstacle2
        obs2->gains.dt /= coeff;
        traj_obs2.line_offset_x = traj_obs1.line_offset_x; 
    }

    node->set_parameter(rclcpp::Parameter("robot.traj.dt", robot->gains.dt));
    node->set_parameter(rclcpp::Parameter("obs1.traj.dt", obs1->gains.dt));
    node->set_parameter(rclcpp::Parameter("obs2.traj.dt", obs2->gains.dt));

    /********************************************************************************/
    /******************************** Path publishing *******************************/
    /********************************************************************************/

    auto path_pub = node->create_publisher<Path>("/robot/path", 1);
    auto path_obs1_pub = node->create_publisher<Path>("/obs1/path", 1);
    auto path_obs2_pub = node->create_publisher<Path>("/obs2/path", 1);
    auto path_timer = node->create_wall_timer(std::chrono::milliseconds{(int)(1000)}, [&]()
    {   
        path_pub->publish(traj.fullPath(node->get_clock()->now()));
        path_obs1_pub->publish(traj_obs1.fullPath(node->get_clock()->now()));
        path_obs2_pub->publish(traj_obs2.fullPath(node->get_clock()->now()));
    });
    
    /********************************************************************************/
    /***************************** Trajectory publishing ****************************/
    /********************************************************************************/

    /* Robot and Trajectory */
    std_msgs::msg::Float64 traj_vel;
    std_msgs::msg::Float64 traj_vel_obs1;
    std_msgs::msg::Float64 traj_vel_obs2;

    traj_vel.data = node->get_parameter(robot->getName() + ".traj.dt").as_double();
    traj_vel_obs1.data = node->get_parameter("obs1.traj.dt").as_double();
    traj_vel_obs2.data = node->get_parameter("obs2.traj.dt").as_double();

    auto traj_vel_pub = node->create_publisher<std_msgs::msg::Float64>("/simulate/traj_dt_robot", 1); 
    auto traj_vel_pub_obs1 = node->create_publisher<std_msgs::msg::Float64>("/simulate/traj_dt_obs1", 1); 
    auto traj_vel_pub_obs2 = node->create_publisher<std_msgs::msg::Float64>("/simulate/traj_dt_obs2", 1); 

    auto sub_traj = 
        node->create_subscription<std_msgs::msg::Float64>("/simulate/traj_dt_robot", 1, [&](std_msgs::msg::Float64::UniquePtr msg)
    {
        traj_vel.data = msg->data;
        robot->gains.dt = msg->data;
    });

    auto robot_timer = node->create_wall_timer(std::chrono::milliseconds{(int)(1000*robot->dt)}, [&]()
    {
        robot->publish(node->get_clock()->now(), traj_vel.data);
        obs1->publish(node->get_clock()->now(), traj_vel_obs1.data);
        obs2->publish(node->get_clock()->now(), traj_vel_obs2.data);

        traj_vel_pub->publish(traj_vel);
        traj_vel_pub_obs1->publish(traj_vel_obs1);
        traj_vel_pub_obs2->publish(traj_vel_obs2);
    });

    rclcpp::spin(node);
}
