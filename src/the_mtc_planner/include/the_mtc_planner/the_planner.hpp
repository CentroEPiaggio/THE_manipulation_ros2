#include <rclcpp/rclcpp.hpp>
// include task constructor stuff
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
// include plnanning scene interface and moveit msg to add collision object
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

// include interfaces
#include "the_interfaces/srv/move_to.hpp"
#include "the_interfaces/srv/visual_end_effector.hpp"
#include "the_interfaces/srv/go_to_group_state.hpp"
#include "the_interfaces/srv/send_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

// include eigen geometry 
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sensor_msgs/msg/joint_state.hpp>
#pragma once

#define INERTIAL_FRAME "world"
#define OBSTACLE_THICKNESS 0.1
#define TABLE_NAME "table"
#define WORKSPACE_NAME "workspace"
#define WORKSPACE_DIMENSIONS {0.5,0.5,0.5}

namespace the_task_generator
{


    namespace mtc = moveit::task_constructor;
    // the ExtrinsicCalibrationTaskGenerator is a ros2 node which provide a point to point planning in world frame.
    // the planning is provided by a the the_inteface MoveTo service
    using PlanningService = the_interfaces::srv::MoveTo;
    using PlanningReq = PlanningService::Request;
    using PlanningRes = PlanningService::Response;
    using ExecTask = std_srvs::srv::Trigger;
    using ExecTaskReq = ExecTask::Request;
    using ExecTaskRes = ExecTask::Response;
    using GroupStateService = the_interfaces::srv::GoToGroupState;
    using GroupStateServiceReq = GroupStateService::Request;
    using GroupStateServiceRes = GroupStateService::Response;
    using VisualEndEffService = the_interfaces::srv::SendPose;
    using VisualEndEffReq = VisualEndEffService::Request;
    using VisualEndEffRes = VisualEndEffService::Response;
    using MoveWS = the_interfaces::srv::SendPose;
    using MoveWSReq = MoveWS::Request;
    using MoveWSRes = MoveWS::Response;
    using TargetPose = geometry_msgs::msg::PoseStamped;
    using MoveitError = moveit::core::MoveItErrorCode;
    using ExecutionService = std_srvs::srv::Trigger;
    using ExecutionServiceReq = ExecutionService::Request;
    using ExecutionServiceRes = ExecutionService::Response;
    using Vector3d = Eigen::Vector3d;
    using Quaterniond = Eigen::Quaterniond;
    using Affine3d = Eigen::Affine3d;
    
    namespace rvt = rviz_visual_tools;
    
    const std::vector<double> default_table_dimensions = {2.0,2.0,0.2};

    class TaskConstructorPlanner : public rclcpp::Node
    {
        public:
            TaskConstructorPlanner(const rclcpp::NodeOptions& options);

            void declare_parameters();

            void get_parameters();
            
            void init_visual_tools();
            
        private:
            
            void add_workspace_collision(moveit_msgs::msg::CollisionObject& msg);

            void ShowErrors(const mtc::Task& t);

            void set_up_planning_scene(std::vector<double> table_dimensions);

            void MoveToPlan(std::shared_ptr<PlanningReq> request, std::shared_ptr<PlanningRes> response );
            
            void GroupStatePlan(GroupStateServiceReq::SharedPtr request, GroupStateServiceRes::SharedPtr response);

            void ExecuteTask(ExecTaskReq::SharedPtr request, ExecTaskRes::SharedPtr response);

            void VisualEndEff(std::shared_ptr<VisualEndEffReq> request, std::shared_ptr<VisualEndEffRes> response); 

            bool BuildGoToTask(TargetPose target_pose);

            bool BuildGroupStateTask(std::string group_state_name);

            bool PlanTask(MoveitError& error_code);

            void move_ws(MoveWSReq::SharedPtr request, MoveWSRes::SharedPtr response);

           

            std::unique_ptr<mtc::Task> task_;

            rclcpp::Service<PlanningService>::SharedPtr motion_service_;
            rclcpp::Service<VisualEndEffService>::SharedPtr visual_end_eff_service_;
            rclcpp::Service<GroupStateService>::SharedPtr group_motion_service_;
            rclcpp::Service<ExecTask>::SharedPtr execute_task_service_;
            rclcpp::Service<MoveWS>::SharedPtr move_ws_service_;

            bool  solution_set_ = false;
            
            std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
            mtc::SolutionBaseConstPtr exec_solution_; 


            // parameters storage
            Affine3d camera_frame_pose_;
                // respect camera frame is define the distnce on the right (-y), on the left (+y), on the front (-x), on the back (-x)
            std::vector<double> workspace_dimensions_camera_frame_ = {0.4,0.4,0.6},table_dimensions_;
            bool gui_debug_, visualize_cartesian_path_;
            std::string group_name_, default_eef_ik_;
            int max_plan_solution_;
            double planning_timeout_;
            std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;

    };


};