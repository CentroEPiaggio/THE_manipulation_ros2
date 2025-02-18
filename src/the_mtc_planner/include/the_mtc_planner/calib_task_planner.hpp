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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

// include eigen geometry 
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#pragma once

#define INERTIAL_FRAME "world"

namespace the_task_generator{


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
    using VisualEndEffService = the_interfaces::srv::VisualEndEffector;
    using VisualEndEffReq = VisualEndEffService::Request;
    using VisualEndEffRes = VisualEndEffService::Response;
    using TargetPose = geometry_msgs::msg::PoseStamped;
    using MoveitError = moveit::core::MoveItErrorCode;
    using ExecutionService = std_srvs::srv::Trigger;
    using ExecutionServiceReq = ExecutionService::Request;
    using ExecutionServiceRes = ExecutionService::Response;
    
    namespace rvt = rviz_visual_tools;
    
    const std::vector<double> default_table_dimensions = {2.0,2.0,0.2};

    class ExtrinsicCalibrationTaskGenerator : public rclcpp::Node
    {
        public:
            ExtrinsicCalibrationTaskGenerator(const rclcpp::NodeOptions& options);

            void get_parameters();
            
            void init_visual_tools();
            
        private:
            

            void ShowErrors(const mtc::Task& t);

            void set_up_planning_scene(std::vector<double> table_dimensions);

            void MoveToPlan(std::shared_ptr<PlanningReq> request, std::shared_ptr<PlanningRes> response );
            
            void GroupStatePlan(GroupStateServiceReq::SharedPtr request, GroupStateServiceRes::SharedPtr response);

            void ExecuteTask(ExecTaskReq::SharedPtr request, ExecTaskRes::SharedPtr response);

            void VisualEndEff(std::shared_ptr<VisualEndEffReq> request, std::shared_ptr<VisualEndEffRes> response); 

            bool BuildGoToTask(TargetPose target_pose);

            bool BuildGroupStateTask(std::string group_state_name);

            bool PlanTask(MoveitError& error_code);

            std::unique_ptr<mtc::Task> task_;

            rclcpp::Service<PlanningService>::SharedPtr motion_service_;
            rclcpp::Service<VisualEndEffService>::SharedPtr visual_end_eff_service_;
            rclcpp::Service<GroupStateService>::SharedPtr group_motion_service_;
            rclcpp::Service<ExecTask>::SharedPtr execute_task_service_;

            bool gui_debug_, visualize_cartesian_path_, solution_set_ = false;
            std::string group_name_, eef_ik_;
            int max_plan_solution_;
            double planning_timeout_;
            std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
            mtc::SolutionBaseConstPtr exec_solution_; 

    };
};
//     class ExtrinsicCalibrationMoveGroupInterface : public rclcpp::Node
//     {
//         public:
//             ExtrinsicCalibrationMoveGroupInterface(const rclcpp::NodeOptions& options);

            
//             void init_planning_pipe()
//             {
//                 this->create_mg_interface();
//                 this->init_visual_tools();
//                 RCLCPP_INFO(this->get_logger(),"Planning Structure initialized");
//             };

//             void prova_call(std::shared_ptr<sensor_msgs::msg::JointState> msg)
//             {
//                 RCLCPP_INFO(this->get_logger(),"ye");
//             };

//         private:

            

//             void MoveToPlanExec(std::shared_ptr<MotionReq> request, std::shared_ptr<MotionRes> response );

//             void VisualEndEff(std::shared_ptr<VisualEndEffReq> request, std::shared_ptr<VisualEndEffRes> response); 

//             void set_up_planning_scene(std::vector<double> table_dimensions);

//             void create_mg_interface();

//             void init_visual_tools();

//             void get_parameters();

//             std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

//             std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;  

//             rclcpp::Service<MotionService>::SharedPtr motion_service_;
//             rclcpp::Service<VisualEndEffService>::SharedPtr visual_end_eff_service_;

//             bool gui_debug_, visualize_cartesian_path_;
//             std::string group_name_, eef_ik_;
//             int max_plan_solution_;
//             double planning_timeout_;

//             rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr prova_; 

//     };

// };