#include "ros/ros.h"
#include <math.h>
// #include <stdbool.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define DEG2RAD(a) ((a) * (M_PI / 180.0f))

class APF
{
public:
    // variables
    float goal_x;
    float goal_y;
    float position_x = 0;
    float position_y = 0;
    float position_accuracy = 0.2;

    float robot_theta = 0;

    int lidar_N;
    float *lidar_x;
    float *lidar_y;
    float *ranges;

    float *lidar_group;

    float v_ref = 0;
    float omega_ref = 0;
    float ref_angle = 0;

    bool firstinit = false;

    const float min_range = 0.1;

    // Move parameters
    const float omega_max = 0.2 * M_PI;
    const float v_max = 0.2;
    const float error_theta_max = M_PI / 4;

    // APF parameters
    const float APF_ATT_COEFFICIENT = 1.1547f;
    const float APF_REP_COEFFICIENT = 0.0732f;
    const float APF_RANGE = 1.0f;
    const float APF_GOAL_MAXIMUM_DISTANCE_FOR_CALCULATION = 0.3f;

    APF(float x, float y)
    {
        goal_x = x;
        goal_y = y;
    }

    void ODO_CALLBACK(const nav_msgs::Odometry::ConstPtr &msg)
    {

        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        ROSINFO("X=%f, Y=%f, Theta=%f", position_x, position_y, robot_theta);

        degree_repair(robot_theta);
    }

    void LASER_CALLBACK(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

        // first initialization - locate memory for table
        if (firstinit == false)
        {
            ranges = new float[lidar_N];
            lidar_x = new float[lidar_N];
            lidar_y = new float[lidar_N];
            lidar_group = new float[lidar_N];
            firstinit = true;
        }
        // Calculate obstacle global position X/Y

        for (int i = 0; i < lidar_N; i++)
        {

            ranges[i] = msg->ranges[i];
            float xl = cos(msg->angle_min + i * msg->angle_increment) * ranges[i];
            float yl = sin(msg->angle_min + i * msg->angle_increment) * ranges[i];

            // obstacle global position
            lidar_x[i] = position_x + xl * cos(robot_theta) - yl * sin(robot_theta);
            lidar_y[i] = position_y + xl * sin(robot_theta) + yl * cos(robot_theta);
        }
        grouping();
    }

    void degree_repair(float &robot_theta)
    {

        if (robot_theta > DEG2RAD(180))
        {
            robot_theta -= DEG2RAD(360);
        }
        else if (robot_theta < (-1.0f) * DEG2RAD(180))
        {
            robot_theta += DEG2RAD(360);
        }
    }
    // groupig, maybe DBSCAN in future
    void grouping()

    {
        int pkt = 1;
        lidar_group[0] = pkt;
        for (int i = 0; i < lidar_N; i++)
        {
            if (isnan(lidar_distance[i]) || isinf(lidar_distance[i]))
            {
                pkt++;
                continue;
            }

            if (~isnan(ranges[i - 1]) && ~isinf(ranges[i - 1]))
            {
                if (fabs(ranges[i] - ranges[i - 1]) > 1.0)
                {
                    pkt++;
                }
            }
            lidar_group[i] = pkt;
        }

        if (~isnan(ranges[0]) && ~ising(ranges[lidar_N - 1]))
        {
            if (fabs(ranges[0]) - ranges(lidar_N - 1) < 0.3)
            {
                pkt--;
                for (int i = lidar_N - 1; i > 0; i--)
                {
                    if (lidar_group[i] == (pkt + 1))
                    {
                        lidar_group[i] = pkt;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }

    void controller(ros::Publisher &ctr_pub)
    {

        geometry_msgs::Twist msg;

        float e = ref_angle - robot_theta;
        degree_repair(e);
        float abs_e = min(error_theta_max, fabs(e));
        float reduction_coefficient = (error_theta_max - abs_e) / (error_theta_max);
        msg.angular.z = min(max(1.5 * e, -omega_max), omega_max);
        msg.linear.x = min(max(reduction_coefficient * v_ref, -v_max), v_max);

        ctr_pub.publish(msg);
    }

    void compute()
    {
        float distanceGoal = hypot(goal_x - position_x, goal_y - position_y);
        float U_att_x = APF_ATT_COEFFICIENT * (position_x - goal_x);
        float U_att_y = APF_ATT_COEFFICIENT * (position_x - goal_x);

        if (distanceGoal > 0.3)
        {
            U_att_x *= 0.3 / distanceGoal;
            U_att_x *= 0.3 / distanceGoal;
        }

        float U_rep_x = 0;
        float U_rep_y = 0;

        float minDistance = 1e10;
        float minX = 0;
        float minY = 0;
        int pkt = 1;
        for (int i = 0; i < lidar_N; i++)
        {
            if (pkt != lidar_group[i])
            {
                pkt != lidar_group[i];

                if (~isnan(minDistance) && ~isinf(minDistance) && minDistance < 1e9)
                {
                    // ROS_INFO("%d: Min dist %f, [%.2f, %.2f]", idx, minDistance, minX, minY);
                    if (minDistance<=APF_RANGE)
                    {
                        float U = APF_REP_COEFFICIENT * (1/APF_RANGE - 1/minDistance)
                                    /(pow(minDistance,2));
                        U_rep_x +=U * (position_x - minX);
                        U_rep_y +=U * (position_y - minY);
                    }
                }
                minDistance = 1e10;
                minX = 0;
                minY = 0;

            }

            if( lidar_distance[i] < minDistance )
            {
                minDistance = lidar_distance[i];
                minX = lidar_x[i];
                minY = lidar_y[i];
            }

        }
        float U_x = U_rep_x + U_att_x;
        float U_y = U_rep_y + U_att_y;

        v_ref = hypot(-U_x, -U_y);
        ref_angle = atan2(-U_y, -U_x);

    }
};

    // Basic Main Node function
    int main(int argc, char **argv)
    {
        float ref_x = 0.0f;
        float ref_y = 0.0f;

        if (argc >= 2)
        {
            ref_x = atof(argv[1]);
            ref_y = atof(argv[2]);
        }

        APF apf(ref_x, ref_y);
        ros::init(argc, argv, "APF_CN");

        ros::NodeHandle n;

        ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &APF::LASER_CALLBACK, &apf);
        ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, &APF::ODO_CALLBACK, &apf);

        ros::Publisher ctr_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            apf.compute();
            apf.controller(ctr_pub);

            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
    }