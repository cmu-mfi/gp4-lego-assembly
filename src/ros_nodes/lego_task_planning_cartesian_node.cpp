#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "Robot.hpp"
#include "Lego.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include <fstream>
#include <unistd.h>

// HEADER FILES FOR SERVICE INTERFACE WITH YK ROBOT
#include "geometry_msgs/Pose.h"
#include "gp4_lego/SetPose.h"

// HEADER FILES FOR API INTERFACE WITH YK ROBOT
#include "yk_api/yk_interface.h"

using namespace std::chrono;

#define ROBOT_DOF 6

gp4_lego::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);

void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

int main(int argc, char **argv)
{
    try
    {
        // A. ROS INITIALIZATION
        //*****************************************************************************************
        ros::init(argc, argv, "task_planning_node");
        ros::NodeHandle nh;
        ros::NodeHandle private_node_handle("~");
        std::string base_frame;
        std::string config_fname, root_pwd;
        float x_home, y_home;

        // get params from launch file
        private_node_handle.param<std::string>("base_frame", base_frame, "world");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        private_node_handle.getParam("config_fname", config_fname);
        private_node_handle.getParam("root_pwd", root_pwd);
        private_node_handle.param<float>("x_home", x_home, 0.3);
        private_node_handle.param<float>("y_home", y_home, 0.0);

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();
        ros::Rate loop_rate(150);

        // B. READ CONFIG FILE
        //*****************************************************************************************
        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        std::string DH_fname = root_pwd + config["DH_fname"].asString();
        std::string DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        std::string DH_tool_assemble_fname = root_pwd + config["DH_tool_assemble_fname"].asString();
        std::string DH_tool_disassemble_fname = root_pwd + config["DH_tool_disassemble_fname"].asString();
        std::string robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        std::string gazebo_env_setup_fname = root_pwd + config["env_setup_fname"].asString();
        std::string task_fname = root_pwd + config["task_graph_fname"].asString();
        bool infinite_tasks = config["Infinite_tasks"].asBool();
        bool assemble = config["Start_with_assemble"].asBool();
        bool use_robot = config["Use_robot"].asBool();

        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        // C. GP4_LEGO INITIALIZATION
        //*****************************************************************************************
        gp4_lego::lego::Lego_Gazebo::Ptr lego_gazebo_ptr = std::make_shared<gp4_lego::lego::Lego_Gazebo>();
        lego_gazebo_ptr->setup(gazebo_env_setup_fname, assemble, task_json);

        gp4_lego::robot::Robot::Ptr robot = std::make_shared<gp4_lego::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->set_DH_tool(DH_tool_fname);
        robot->set_DH_tool_assemble(DH_tool_assemble_fname);
        robot->set_DH_tool_disassemble(DH_tool_disassemble_fname);
        robot->print_robot_property();
        Eigen::Matrix<double, 3, 1> cart_goal = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd twist_R(3, 3);
        Eigen::MatrixXd twist_T(4, 4);
        Eigen::MatrixXd cart_T(4, 4);

        int twist_deg = 12;
        double incremental_deg = twist_deg;
        int twist_num = twist_deg / incremental_deg;

        int num_tasks = task_json.size();
        int twist_idx = 0;
        double ik_step = 10e-4;
        int max_iter = 10e7;
        int grab_brick;
        std_msgs::Float32MultiArray goal_msg;
        Eigen::Matrix4d cart_T_goal;

        int task_idx;

        if (assemble)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }

        // D. HOME POSE
        //*****************************************************************************************
        Eigen::MatrixXd home_q(ROBOT_DOF, 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0; // Home
        Eigen::Matrix4d home_T = gp4_lego::math::FK(home_q, robot->robot_DH(), robot->robot_base(), false);
        home_T.col(3) << x_home, y_home, 0.5, 1; // Home X, Y, Z in base frame of the Flange (tool0) ??TODO??
        home_q = gp4_lego::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                    robot->robot_DH(), robot->robot_base(), 0, max_iter, ik_step);

        // E. INITIALIZE ROS INTERFACES
        //*****************************************************************************************
        /** TODO: USE SERVICE INTERFACE TO SEND GOAL TO ROBOT
         *  Requires service server node, i.e., yk_tasks/yk_tasks, to be running
         */
        ros::ServiceClient client = nh.serviceClient<gp4_lego::SetPose>("yk_set_pose");
        gp4_lego::SetPose srv;

        /** TODO: USE YK_API TO SEND GOAL TO ROBOT
         *  Requires CMakeLists.txt to be modified to include yk_api
         */
        // MFI::YK_Interface yk_interface(namespace);

        // VIRTUAL CONTROLLER INTERFACE ??TODO??
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("goal", ROBOT_DOF);
        ros::Subscriber robot_state_sub = nh.subscribe("robot_state", ROBOT_DOF * 3, robotStateCallback);

        // F. MAIN LOOP
        //*****************************************************************************************
        int pre_mode = -1;
        gp4_lego::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = true;

        int mode = 0; // 0:home 1:pick_up 2:pick_down, 3:pick_twist 4:pick_twist_up 5:home
                      // 6:drop_up 7:drop_down 8:drop_twist 9:drop_twist_up 10:done

        while (ros::ok)
        {
            // F.1. DETERMINE AND CALCULATE GOAL
            //*************************************************************************************
            auto loop_start = high_resolution_clock::now();
            robot->set_robot_q(robot_q);
            robot->set_robot_qd(robot_qd);
            robot->set_robot_qdd(robot_qdd);
            if (mode >= 3 && mode <= 7)
            {
                if (pre_mode != mode)
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, mode + twist_idx, brick_name);
                }
                else
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, 0, brick_name);
                }
            }
            pre_mode = mode;
            if ((use_robot && move_on_to_next) || (!use_robot && robot->reached_goal(cur_goal) && robot->is_static()))
            {
                if (mode == 7)
                {
                    lego_gazebo_ptr->update_brick_connection();
                }
                if (mode == 10)
                {
                    if (task_idx >= num_tasks && !infinite_tasks && assemble)
                    {
                        break;
                    }
                    else if (task_idx <= 1 && !infinite_tasks && !assemble)
                    {
                        break;
                    }
                    else if (task_idx >= num_tasks && assemble)
                    {
                        task_idx++;
                        assemble = !assemble;
                    }
                    else if (task_idx <= 1 && !assemble)
                    {
                        task_idx--;
                        assemble = !assemble;
                    }
                    mode = 0;
                    if (assemble)
                    {
                        task_idx++;
                    }
                    else
                    {
                        task_idx--;
                    }
                }
                else if (mode == 3 || mode == 8)
                {
                    twist_idx++;
                    if (twist_idx == twist_num)
                    {
                        mode++;
                        twist_idx = 0;
                    }
                }
                else
                {
                    mode++;
                }
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                if (mode == 0 || mode == 5 || mode == 10)
                {
                    cur_goal = home_q;
                }
                else if (mode == 1)
                {
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    brick_name = lego_gazebo_ptr->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
                    grab_brick = 1;
                    lego_gazebo_ptr->calc_brick_grab_pose(brick_name, assemble, grab_brick,
                                                          cur_graph_node["x"].asInt(),
                                                          cur_graph_node["y"].asInt(),
                                                          cur_graph_node["z"].asInt(),
                                                          cur_graph_node["ori"].asInt(),
                                                          cur_graph_node["press_side"].asInt(), cart_T);
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal = gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, up_T, robot->robot_DH_tool(),
                    //                                                       robot->robot_base_inv(), robot->robot_tool_inv(), 0, IK_status);
                    if (!assemble && lego_gazebo_ptr->brick_instock(brick_name))
                    {
                        cur_goal = home_q;
                        mode = 10;
                    }
                }
                else if (mode == 2)
                {
                    cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                }
                else if (mode == 3)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad),
                        0, 1, 0,
                        -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_disassemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool_disassemble(), robot->robot_base(), 0, max_iter, ik_step);
                }
                else if (mode == 4 || mode == 9)
                {
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T(2, 3) = cart_T(2, 3) + 0.015;
                    cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_step);
                }
                else if (mode == 6)
                {
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    grab_brick = 0;
                    lego_gazebo_ptr->calc_brick_grab_pose(brick_name, assemble, 0,
                                                          cur_graph_node["x"].asInt(),
                                                          cur_graph_node["y"].asInt(),
                                                          cur_graph_node["z"].asInt(),
                                                          cur_graph_node["ori"].asInt(),
                                                          cur_graph_node["press_side"].asInt(), cart_T);
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal = gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                }
                else if (mode == 7)
                {
                    // cart_T(2, 3) = cart_T(2, 3) - 0.0035;
                    cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool(),
                    //                                                       robot->robot_base_inv(), robot->robot_tool_inv(),0,IK_status);
                }
                else if (mode == 8)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad),
                        0, 1, 0,
                        -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                  robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_step);
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool_assemble(),
                    //                                                       robot->robot_base_inv(), robot->robot_tool_assemble_inv(),0,IK_status);
                }
            }

            cart_T_goal = gp4_lego::math::FK(cur_goal, robot->robot_DH(), robot->robot_base(), false);
            Eigen::Matrix3d goal_rot = cart_T_goal.block(0, 0, 3, 3);
            Eigen::Quaterniond quat(goal_rot);

            // F.2. PUBLISH GOAL TO ROBOT CONTROLLERS (REAL AND VIRTUAL)
            //*************************************************************************************
            // F.2.1. Simulation command
            goal_msg.data.clear();
            for (int j = 0; j < ROBOT_DOF; j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);

            // F.2.2. Robot command
            if (use_robot)
            {
                geometry_msgs::Pose goal_pose;
                if (move_on_to_next)
                {
                    goal_pose.position.x = cart_T_goal(0, 3);
                    goal_pose.position.y = cart_T_goal(1, 3);
                    goal_pose.position.z = cart_T_goal(2, 3);
                    goal_pose.orientation.x = quat.x();
                    goal_pose.orientation.y = quat.y();
                    goal_pose.orientation.z = quat.z();
                    goal_pose.orientation.w = quat.w();
                    move_on_to_next = false;
                    ROS_INFO_STREAM("Sending pose: " << goal_pose);
                }

                /** TODO: USE SERVICE INTERFACE TO SEND GOAL TO ROBOT
                 */
                srv.request.base_frame = "base_link";
                srv.request.pose = goal_pose;
                if (client.call(srv))
                {
                    move_on_to_next = true;
                    ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                }

                /** TODO: USE YK_API TO SEND GOAL TO ROBOT
                 */
                // yk_interface.goToPoseGoal(goal_pose);
            }
            // END OF LOOP
            //*************************************************************************************
            // ros::spinOnce();
        }

        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}
