#ifndef TRAJ_H
#define TRAJ_H

#include <mobro_sim/utils.h>

struct Traj
{ 
    int path_num;

    // Common
    double w = 0.1; // frequency

    // Circular path
    double r = 7.5; // ray
    double b = r;   // y-offset
    double a = 0.0; // x-offset

    // Sinusoidal Path
    double amp = 5.;
    double phi = 10.;
    double freq = 0.75;

    // Lines (horizontal and vertical)
    double line_offset_x = 0.;
    double line_offset_y = 0.;
    double coeff_x = 10.;
    double coeff_y = 10.;
    double slope = 0.;
    
    /*!
     * Produce the full path.
     * @param now current clock time.
     */
    inline Path fullPath(const rclcpp::Time &now) const
    {
        Path path;

        if(path.poses.empty())
        {
            path.header.frame_id = "map";
            auto t{-M_PI/w};
            const auto dt{0.1*M_PI};

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.orientation.w = 1.;

            while(t < M_PI/w)
            {
                const auto [p,v,acc] = currentPath(t); {}
                pose.pose.position.x = p[0];
                pose.pose.position.y = p[1];
                t += dt;
                path.poses.push_back(pose);
            }
        }

        for(auto &pose: path.poses)
        pose.header.stamp = now;

        return path;
    }

    /*!
     * Produce a trajectory along the path.
     * @param angle trajectory angle.
     * @param dt trajectory velocity.
     */
    std::tuple<Vector2d, Vector2d, Vector2d> currentTraj(double& angle, double dt) const
    {
        const auto c{cos(angle)};
        const auto s{sin(angle)};

        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3,2);

        if (path_num == 1) matrix = circularPath(c,s); 
        else if (path_num == 2) matrix = sinPath(angle,c,s); 
        else if (path_num == 3) matrix = horizontalLinePath(c,s); 
        else if (path_num == 4) matrix = verticalLinePath(c,s); 

        angle += w * dt; // update 

        pd = matrix.row(0);
        vd = matrix.row(1);
        ad = matrix.row(2);

        return std::make_tuple(pd,vd,ad);
    }

    /*!
     * Produce a path.
     * @param t current clock time.
     */
    std::tuple<Vector2d, Vector2d, Vector2d> currentPath(double t) const
    {   
        const auto c{cos(w*t)};
        const auto s{sin(w*t)};

        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3,2);
        
        if (path_num == 1) matrix = circularPath(c,s); 
        else if (path_num == 2) matrix = sinPath(t,c,s); 
        else if (path_num == 3) matrix = horizontalLinePath(c,s); 
        else if (path_num == 4) matrix = verticalLinePath(c,s); 

        pd = matrix.row(0);
        vd = matrix.row(1);
        ad = matrix.row(2);

        return std::make_tuple(pd,vd,ad);
    }

    /*!
     * Produce a circular path.
     * @param c angle cosine.
     * @param s angle sine. 
     */
    Eigen::MatrixXd circularPath(double c, double s) const
    {
        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix(3,2);

        pd[0] = a + r * c;  // x-coordinate
        pd[1] = b + r * s;  // y-coordinate

        vd[0] = -r*s;       // x-direction
        vd[1] = +r*c;       // y-direction

        ad[0] = -r*c;
        ad[1] = -r*s;

        matrix.block(0,0,1,2) = pd.transpose();
        matrix.block(1,0,1,2) = vd.transpose();
        matrix.block(2,0,1,2) = ad.transpose();

        return matrix;
    }

    /*!
     * Produce a sinusoidal path.
     * @param t time.
     * @param s angle sine. 
     */
    Eigen::MatrixXd sinPath(double t, double c, double s) const
    {
        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix(3,2);

        pd[0] = t;                          // x-coordinate
        pd[1] = amp*sin(freq*t+phi);        // y-coordinate

        vd[0] = 1;                          // x-direction
        vd[1] = amp*cos(freq*t+phi)*freq;   // y-direction

        ad[0] = -r*c;
        ad[1] = -r*s;

        matrix.block(0,0,1,2) = pd.transpose();
        matrix.block(1,0,1,2) = vd.transpose();
        matrix.block(2,0,1,2) = ad.transpose();

        return matrix;
    }

    /*!
     * Produce an horizontal line.
     * @param c cosine.
     * @param s sine.
     */
    Eigen::MatrixXd horizontalLinePath(double c, double s) const
    {
        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix(3,2);
        
        pd[0] = coeff_x*c + line_offset_x;  // x-coordinate
        pd[1] = line_offset_y;              // y-coordinate

        vd[0] = -coeff_x*s;                 // x-direction
        vd[1] = 0;                          // y-direction

        ad[0] = -coeff_x*c;
        ad[1] = 0;

        matrix.block(0,0,1,2) = pd.transpose();
        matrix.block(1,0,1,2) = vd.transpose();
        matrix.block(2,0,1,2) = ad.transpose();

        return matrix;
    }

    /*!
     * Produce a vertical line.
     * @param c cosine.
     * @param s sine.
     */
    Eigen::MatrixXd verticalLinePath(double c, double s) const
    {
        Vector2d pd, vd, ad;
        Eigen::MatrixXd matrix(3,2);
        
        pd[0] = line_offset_x;              // x-coordinate
        pd[1] = coeff_y*s + line_offset_y;  // y-coordinate

        vd[0] = 0;                          // x-direction
        vd[1] = coeff_y*c;                  // y-direction

        ad[0] = 0;
        ad[1] = -coeff_y*s;

        matrix.block(0,0,1,2) = pd.transpose();
        matrix.block(1,0,1,2) = vd.transpose();
        matrix.block(2,0,1,2) = ad.transpose();

        return matrix;
    }
};

#endif // TRAJ_H
