#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "gazebo_msgs/SetModelState.h"

namespace gp4_lego
{
namespace lego
{
struct lego_brick{
    std::string brick_name;
    int height;
    int width;
    double x;
    double y;
    double z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
    double cur_x;
    double cur_y;
    double cur_z;
    int press_side;
    Eigen::Quaterniond cur_quat;
    bool in_stock;
    std::map<std::string, std::string> top_connect;
    std::map<std::string, std::string> bottom_connect;
};

class Lego_Gazebo
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego_Gazebo> Ptr;
        typedef std::shared_ptr<Lego_Gazebo const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        std::map<std::string, lego_brick> brick_map;
        ros::ServiceClient client; 
        gazebo_msgs::SetModelState setmodelstate;
        double brick_height_m = 0.0096;
        double brick_len_offset = 0.0002;
        double P_len = 0.008;
        double EPS = 0.0001;
        double knob_height = 0.0017;
        double storage_plate_x = 0.0;
        double storage_plate_y = 0.0;
        double storage_plate_z = 0.0;
        int storage_plate_width = 0;
        int storage_plate_height = 0;
        double assemble_plate_x = 0.0;
        double assemble_plate_y = 0.0;
        double assemble_plate_z = 0.0;
        int assemble_plate_width = 0;
        int assemble_plate_height = 0;

        void update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT);
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        void calc_brick_loc(const std::string name, const double& ref_x, const double& ref_y, const int& orientation,
                            const int& plate_width, const int& plate_height, const int& brick_loc_x, const int& brick_loc_y, 
                            double& out_x, double& out_y);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        bool get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);

    public:
        Lego_Gazebo();
        ~Lego_Gazebo(){}
                
        // Operations
        void setup(const std::string& env_setup_fname, const bool& assemble, const Json::Value& task_json);
        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const int& task_mode, const std::string& brick_name);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, Eigen::MatrixXd& T);
        int brick_instock(const std::string& name) {return brick_map[name].in_stock;};
};
}
}