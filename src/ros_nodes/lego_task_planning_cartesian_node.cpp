#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "Robot.hpp"
#include "Lego.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <gp4_lego/MakeMove.h>
#include <fstream>
#include <unistd.h>
using namespace std::chrono;


gp4_lego::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);


void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "task_planning_node");
        ros::NodeHandle nh("~");
        // ros::NodeHandle nh;
        ros::NodeHandle private_node_handle("~");

        std::string base_frame;
        private_node_handle.param<std::string>("base_frame", base_frame, "world");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, root_pwd, task_fname, DH_fname, DH_tool_fname, DH_tool_assemble_fname, DH_tool_disassemble_fname, 
                    robot_base_fname, gazebo_env_setup_fname;
        bool IK_status;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);
        // config_fname = "/home/mfi/repos/ros1_ws/src/gp4-lego-assembly/config/user_config.json";
        // root_pwd = "/home/mfi/repos/ros1_ws/src/gp4-lego-assembly/";
        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        DH_tool_assemble_fname = root_pwd + config["DH_tool_assemble_fname"].asString();
        DH_tool_disassemble_fname = root_pwd + config["DH_tool_disassemble_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        gazebo_env_setup_fname = root_pwd + config["env_setup_fname"].asString();
        task_fname = root_pwd + config["task_graph_fname"].asString();
        bool infinite_tasks = config["Infinite_tasks"].asBool();
        bool assemble = config["Start_with_assemble"].asBool();
        bool use_robot = config["Use_robot"].asBool();

        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;
        ros::Rate loop_rate(150);

        gp4_lego::lego::Lego_Gazebo::Ptr lego_gazebo_ptr = std::make_shared<gp4_lego::lego::Lego_Gazebo>();
        lego_gazebo_ptr->setup(gazebo_env_setup_fname, assemble, task_json);
      
        int robot_dof = 6;
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
        int twist_idx = 0;
        int twist_num = twist_deg / incremental_deg;
        int grab_brick;
       
        ros::ServiceClient client = nh.serviceClient<gp4_lego::MakeMove>("make_a_move");
        gp4_lego::MakeMove srv;

        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("goal", robot_dof);
        ros::Subscriber robot_state_sub = nh.subscribe("robot_state", robot_dof * 3, robotStateCallback);
        std_msgs::Float32MultiArray goal_msg;
        
        int num_tasks = task_json.size();
        double ik_iter_num = 0.02;
        int max_iter = 100000;
        Eigen::Matrix4d cart_T_goal;
        Eigen::MatrixXd home_q(robot_dof, 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0; // Home
        Eigen::Matrix4d home_T = gp4_lego::math::FK(home_q, robot->robot_DH(), robot->robot_base(), false);
        home_T.col(3) << 0.3, 0, 0.5, 1;
        home_q = gp4_lego::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH(), robot->robot_base(), 0, max_iter, ik_iter_num); 

        int mode = 0; // 0: home, 1: pick up, 2, pick down 3: pick twist 4: pick twist up 5: home
                      // 6: drop up 7: drop down 8: drop twist 9: drop twist up 10: done
        int task_idx;
        if(assemble)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }
        
        int pre_mode = -1;
        gp4_lego::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = true;

        while(ros::ok)
        {
            auto loop_start = high_resolution_clock::now();
            robot->set_robot_q(robot_q);
            robot->set_robot_qd(robot_qd);
            robot->set_robot_qdd(robot_qdd);
            if(mode >= 3 && mode <= 7)
            {
                if(pre_mode != mode)
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, mode + twist_idx, brick_name);
                }
                else
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, 0, brick_name);
                }
            }
            pre_mode = mode;
            if((use_robot && move_on_to_next) || (!use_robot && robot->reached_goal(cur_goal) && robot->is_static()))
            {
                if(mode == 7)
                {
                    lego_gazebo_ptr->update_brick_connection();
                }
                if(mode == 10){
                    if(task_idx >= num_tasks && !infinite_tasks && assemble)
                    {
                        break;
                    }
                    else if(task_idx <= 1 && !infinite_tasks && !assemble)
                    {
                        break;
                    }
                    else if(task_idx >= num_tasks && assemble)
                    {
                        task_idx ++;
                        assemble = !assemble;
                    }
                    else if(task_idx <= 1 && !assemble)
                    {
                        task_idx --;
                        assemble = !assemble;
                    }
                    mode = 0;
                    if(assemble)
                    {
                        task_idx ++;
                    }
                    else
                    {
                        task_idx --;
                    }
                }
                else if(mode == 3 || mode == 8)
                {
                    twist_idx ++;
                    if(twist_idx == twist_num)
                    {
                        mode ++;
                        twist_idx = 0;
                    }
                }
                else{
                    mode ++;
                }
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                if(mode == 0 || mode == 5 || mode == 10)
                {
                    cur_goal = home_q;
                }
                else if(mode == 1)
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
                    cur_goal =  gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, up_T, robot->robot_DH_tool(), 
                    //                                                       robot->robot_base_inv(), robot->robot_tool_inv(), 0, IK_status);
                    if(!assemble && lego_gazebo_ptr->brick_instock(brick_name))
                    {
                        cur_goal = home_q;
                        mode = 10;
                    }
                }
                else if(mode == 2)
                {
                    cur_goal =  gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_iter_num);
                }
                else if(mode == 3)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_disassemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool_disassemble(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                }
                else if(mode == 4 || mode == 9)
                {
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T(2, 3) = cart_T(2, 3) + 0.015;
                    cur_goal =  gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                }
                else if(mode == 6)
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
                    cur_goal =  gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                }
                else if(mode == 7)
                {
                    // cart_T(2, 3) = cart_T(2, 3) - 0.0035;
                    cur_goal =  gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool(), 
                    //                                                       robot->robot_base_inv(), robot->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 8)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                   robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_iter_num); 
                    // cur_goal =  gp4_lego::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool_assemble(), 
                    //                                                       robot->robot_base_inv(), robot->robot_tool_assemble_inv(),0,IK_status);
                }
            }

            cart_T_goal = gp4_lego::math::FK(cur_goal, robot->robot_DH(), robot->robot_base(), false);
            Eigen::Matrix3d goal_rot = cart_T_goal.block(0, 0, 3, 3);
            Eigen::Quaterniond quat(goal_rot);

            // Simulation command
            goal_msg.data.clear();
            for(int j=0; j<robot_dof; j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);

            // Robot command
            if(use_robot)
            {
                if(move_on_to_next)
                {
                    geometry_msgs::Pose goal_pose;
                    goal_pose.position.x = cart_T_goal(0, 3);
                    goal_pose.position.y = cart_T_goal(1, 3);
                    goal_pose.position.z = cart_T_goal(2, 3);
                    goal_pose.orientation.x = quat.x();
                    goal_pose.orientation.y = quat.y();
                    goal_pose.orientation.z = quat.z();
                    goal_pose.orientation.w = quat.w();
                    move_on_to_next = false;
                    srv.request.base_frame = "base_link";
                    srv.request.pose = goal_pose;
                    ROS_INFO_STREAM("Sending pose: " << goal_pose);
                }
                if(client.call(srv))
                {
                    move_on_to_next = true;
                    ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                }
            }
            // ros::spinOnce();
        }	    
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



