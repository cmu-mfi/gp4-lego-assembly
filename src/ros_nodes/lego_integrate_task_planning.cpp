#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "Robot.hpp"
#include "Lego.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64.h"
#include <geometry_msgs/WrenchStamped.h>
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
int task_type_val = 0;
int assembly_task_val = 0;
int start_task_val = 0;
int executing = 0;
gp4_lego::math::VectorJd assembly_plate_pose_val = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd kit_plate_pose_val = Eigen::MatrixXd::Zero(6, 1);
gp4_lego::math::VectorJd fts_val = Eigen::MatrixXd::Zero(6, 1);

void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

void taskTypeCallback(const std_msgs::Int64::ConstPtr &msg)
{
    if(!executing){
        task_type_val = msg->data;
    }
}

void assemblyTaskCallback(const std_msgs::Int64::ConstPtr &msg)
{
    if(!executing){
        assembly_task_val = msg->data;
    }
}

void startTaskCallback(const std_msgs::Int64::ConstPtr &msg)
{
    start_task_val = msg->data;
}

void assemblyPlateStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    assembly_plate_pose_val << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

void kitPlateStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    kit_plate_pose_val << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

void ftsCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    fts_val << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
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
        unsigned int second = 1000000;
        usleep(10 * second);

        // get params from launch file
        private_node_handle.param<std::string>("base_frame", base_frame, "world");
        ROS_INFO_STREAM("namespace of task_planning nh = " << nh.getNamespace());
        private_node_handle.getParam("config_fname", config_fname);
        private_node_handle.getParam("root_pwd", root_pwd);
        private_node_handle.param<float>("x_home", x_home, 0.3);
        private_node_handle.param<float>("y_home", y_home, 0.0);
        ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

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
        assembly_task_val = config["assembly_task_idx"].asInt64();
        std::string task_fname;
        std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();
        bool infinite_tasks = false; //config["Infinite_tasks"].asBool();
        task_type_val = config["Start_with_assemble"].asInt64();
        bool use_robot = config["Use_robot"].asBool();
        bool use_ik = config["IK"]["Use_IK"].asBool();
        
        if(assembly_task_val == 0)
        {
            task_fname = root_pwd + "/config/assembly_tasks/human.json";
        }
        else if(assembly_task_val == 1)
        {
            task_fname = root_pwd + "/config/assembly_tasks/heart.json";
        }
        else if(assembly_task_val == 2 && task_type_val)
        {
            task_fname = root_pwd + "/config/assembly_tasks/wtts.json";
        }
        else if(assembly_task_val == 2 && !task_type_val)
        {
            task_fname = root_pwd + "/config/assembly_tasks/wtts_disassemble.json";
        }
        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        // C. GP4_LEGO INITIALIZATION
        //*****************************************************************************************
        gp4_lego::lego::Lego_Gazebo::Ptr lego_gazebo_ptr = std::make_shared<gp4_lego::lego::Lego_Gazebo>();
        lego_gazebo_ptr->setup(gazebo_env_setup_fname, lego_lib_fname, task_type_val, task_json, 
                               DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
                               robot_base_fname, 1, set_state_client);
        assembly_plate_pose_val << lego_gazebo_ptr->assemble_plate_x(), lego_gazebo_ptr->assemble_plate_y(), lego_gazebo_ptr->assemble_plate_z(), 
                                   lego_gazebo_ptr->assemble_plate_roll(), lego_gazebo_ptr->assemble_plate_pitch(), lego_gazebo_ptr->assemble_plate_yaw();
        kit_plate_pose_val << lego_gazebo_ptr->storage_plate_x(), lego_gazebo_ptr->storage_plate_y(), lego_gazebo_ptr->storage_plate_z(),
                              lego_gazebo_ptr->storage_plate_roll(), lego_gazebo_ptr->storage_plate_pitch(), lego_gazebo_ptr->storage_plate_yaw();

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
        double ik_step = 10e-3;
        int max_iter = 10e6;
        int grab_brick;
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Int64 exec_msg;
        Eigen::Matrix4d cart_T_goal;

        int task_idx;

        if (task_type_val)
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
        home_T.col(3) << x_home, y_home, 0.4, 1; // Home X, Y, Z in base frame of the Flange (tool0) ??TODO??
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
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("sim/gp4_lego_bringup/robot_goal", ROBOT_DOF);
        ros::Subscriber robot_state_sub = nh.subscribe("sim/gp4_lego_bringup/robot_state", ROBOT_DOF * 3, robotStateCallback);
        
        // Communication with other integrated modules
        ros::Publisher exec_status_pub = nh.advertise<std_msgs::Int64>("execution_status", 1); // 0: executing 1: idle
        ros::Subscriber task_type_sub = nh.subscribe("task_type", 1, taskTypeCallback); // Assemble: 1/Disassemble: 0
        ros::Subscriber assembly_task_sub = nh.subscribe("assembly_task", 1, assemblyTaskCallback); // Assembly tasks: 0:human, 1: heart, 2:stairs
        ros::Subscriber start_task_sub = nh.subscribe("start_task", 1, startTaskCallback); // 0: no new task, 1: new task comming.
        ros::Subscriber assembly_plate_state_sub = nh.subscribe("assembly_plate_state", 6, assemblyPlateStateCallback); // X, Y, Yaw
        ros::Subscriber kit_plate_state_sub = nh.subscribe("kit_plate_state", 6, kitPlateStateCallback); // X, Y, Yaw
        ros::Subscriber fts_sub = nh.subscribe("/fts", 1, ftsCallback);

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
            
            // new task coming
            if(!executing && start_task_val)
            {
                ROS_INFO_STREAM("\n\nReset environment!");
                executing = 1;
                if(assembly_task_val == 0)
                {
                    task_fname = root_pwd + "/config/assembly_tasks/human.json";
                }
                else if(assembly_task_val == 1)
                {
                    task_fname = root_pwd + "/config/assembly_tasks/heart.json";
                }
                else if(assembly_task_val == 2 && task_type_val)
                {
                    task_fname = root_pwd + "/config/assembly_tasks/wtts.json";
                }
                else if(assembly_task_val == 2 && !task_type_val)
                {
                    task_fname = root_pwd + "/config/assembly_tasks/wtts_disassemble.json";
                }
                std::ifstream task_file(task_fname, std::ifstream::binary);
                task_file >> task_json;
                lego_gazebo_ptr->set_assemble_plate_pose(assembly_plate_pose_val[0], assembly_plate_pose_val[1], assembly_plate_pose_val[5]);
                lego_gazebo_ptr->set_storage_plate_pose(kit_plate_pose_val[0], kit_plate_pose_val[1], kit_plate_pose_val[5]);
                lego_gazebo_ptr->setup(gazebo_env_setup_fname, lego_lib_fname, task_type_val, task_json, 
                                       DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
                                       robot_base_fname, 0, set_state_client);
                                       
                num_tasks = task_json.size();
                if (task_type_val)
                {
                    task_idx = 1;
                    ROS_INFO_STREAM("Assembling!");
                }
                else
                {
                    task_idx = num_tasks;
                    ROS_INFO_STREAM("Disassembling!");
                }
                ROS_INFO_STREAM("Num steps: " << num_tasks);
                pre_mode = -1;
                cur_goal = home_q;
                move_on_to_next = true;
                
                mode = 0; // 0:home 1:pick_up 2:pick_down, 3:pick_twist 4:pick_twist_up 5:home
                            // 6:drop_up 7:drop_down 8:drop_twist 9:drop_twist_up 10:done
            }
            if(executing)
            {
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
                        if (task_idx >= num_tasks && task_type_val)
                        {
                            executing = 0;
                        }
                        else if (task_idx <= 1 && !task_type_val)
                        {
                            executing = 0;
                        }
                        mode = 0;
                        if (task_type_val)
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
                        lego_gazebo_ptr->calc_brick_grab_pose(brick_name, task_type_val, grab_brick,
                                                            cur_graph_node["x"].asInt(),
                                                            cur_graph_node["y"].asInt(),
                                                            cur_graph_node["z"].asInt(),
                                                            cur_graph_node["ori"].asInt(),
                                                            cur_graph_node["press_side"].asInt(), cart_T);
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                        }
                        if (!task_type_val && lego_gazebo_ptr->brick_instock(brick_name))
                        {
                            cur_goal = home_q;
                            mode = 10;
                        }
                    }
                    else if (mode == 2)
                    {
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                        }
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
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool_disassemble(), robot->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 4 || mode == 9)
                    {
                        cart_T = gp4_lego::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 6)
                    {
                        auto cur_graph_node = task_json[std::to_string(task_idx)];
                        grab_brick = 0;
                        lego_gazebo_ptr->calc_brick_grab_pose(brick_name, task_type_val, 0,
                                                            cur_graph_node["x"].asInt(),
                                                            cur_graph_node["y"].asInt(),
                                                            cur_graph_node["z"].asInt(),
                                                            cur_graph_node["ori"].asInt(),
                                                            cur_graph_node["press_side"].asInt(), cart_T);
                        Eigen::Matrix4d up_T = cart_T;
                        up_T(2, 3) = up_T(2, 3) + 0.015;
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 7)
                    {
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool(), robot->robot_base(), 0, max_iter, ik_step);
                        }
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
                        if(use_ik)
                        {
                            cur_goal = gp4_lego::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robot->robot_DH_tool_assemble(), robot->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                }

                cart_T_goal = gp4_lego::math::FK(cur_goal, robot->robot_DH(), robot->robot_base(), false);
                Eigen::Matrix3d goal_rot = cart_T_goal.block(0, 0, 3, 3);
                Eigen::Quaterniond quat(goal_rot);
                
                goal_msg.data.clear();
                for (int j = 0; j < ROBOT_DOF; j++)
                {
                    goal_msg.data.push_back(cur_goal(j));
                }
                goal_pub.publish(goal_msg);

                // F.2.2. Robot command
                if (use_robot)
                {
                    auto motion_start = high_resolution_clock::now();
                    auto motion_end = high_resolution_clock::now();
                    // auto motion_start, motion_end;
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
                        motion_start = high_resolution_clock::now();
                    }
                    srv.request.base_frame = "base_link";
                    srv.request.pose = goal_pose;
                    if (client.call(srv))
                    {
                        move_on_to_next = true;
                        ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                        motion_end = high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(motion_end - motion_start);
                        ROS_INFO_STREAM("Motion Execution time: " << duration.count() / 1000000.0 << " s");
                    }
                }
            }
            exec_msg.data = executing;
            exec_status_pub.publish(exec_msg);
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
