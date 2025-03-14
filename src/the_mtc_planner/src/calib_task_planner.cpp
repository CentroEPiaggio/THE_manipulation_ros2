#include "the_mtc_planner/calib_task_planner.hpp"
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/utils/moveit_error_code.h>
#include <string>
#include <memory>
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>
namespace the_task_generator{

    using std::placeholders::_1;
    using std::placeholders::_2;
    using Stage = mtc::Stage;
    using SolutionService = moveit_task_constructor_msgs::srv::GetSolution;
    using SolutionServiceRequest = SolutionService::Request;
    using SolutionServiceResponse = SolutionService::Response;
     
    void ExtrinsicCalibrationTaskGenerator::ShowErrors(const mtc::Task& t)
    {
        std::ostringstream os;
        t.printState(os);
        t.explainFailure(os);
        RCLCPP_INFO(this->get_logger(), "%s", os.str().c_str());
        
    };

    ExtrinsicCalibrationTaskGenerator::ExtrinsicCalibrationTaskGenerator(const rclcpp::NodeOptions& options):
    rclcpp::Node("the_calibration_planner", options)
    {
        // declare planning parameter
        if(!this->has_parameter("group_name"))
            declare_parameter<std::string>("group_name","ur_manipulator");
        if(!this->has_parameter("eef_ik"))
            declare_parameter<std::string>("eef_ik","fixed_fingertip");
        if(!this->has_parameter("max_plan_solution"))
            declare_parameter<int>("max_plan_solution",2);
        if(!this->has_parameter("planning_timeout"))
            declare_parameter<double>("planning_timeout",30.0);
        if(!this->has_parameter("visualize_cartesian_path"))
            declare_parameter<bool>("visualize_cartesian_path",true);
        if(!this->has_parameter("table_dimensions"))
            declare_parameter<std::vector<double>>("table_dimensions",std::vector<double>());
        if(!this->has_parameter("camera_frame_position"))
            declare_parameter<std::vector<double>>("camera_frame_position",std::vector<double>({0.0,0.0,0.0}));
        if(!this->has_parameter("camera_frame_orientation"))
            declare_parameter<std::vector<double>>("camera_frame_orientation",std::vector<double>({1.0,0.0,0.0,0.0}));
        
        this->get_parameters();

        // create service for motion
        motion_service_ = this->create_service<PlanningService>(
            "~/target_pose_plan",
            std::bind(&ExtrinsicCalibrationTaskGenerator::MoveToPlan,this,_1,_2)
        );

        execute_task_service_ = this->create_service<ExecTask>(
            "~/exec_planned_task",
            std::bind(&ExtrinsicCalibrationTaskGenerator::ExecuteTask,this,_1,_2)
        );

        // create service for visualize end effector
        visual_end_eff_service_ = this->create_service<VisualEndEffService>(
            "~/visual_end_eff",
            std::bind(&ExtrinsicCalibrationTaskGenerator::VisualEndEff,this,_1,_2)
        );

        group_motion_service_ = this->create_service<GroupStateService>(
            "~/group_state_plan",
            std::bind(&ExtrinsicCalibrationTaskGenerator::GroupStatePlan,this,_1,_2)
        );
        RCLCPP_INFO(this->get_logger(),"Extrinsic Calibration Task Generator Node Created");

        
    }

    void ExtrinsicCalibrationTaskGenerator::VisualEndEff(
        std::shared_ptr<VisualEndEffReq> request,
        std::shared_ptr<VisualEndEffRes> response
    )
    {
        RCLCPP_INFO(this->get_logger(),"Received request to visualize end effector");
        visual_tools_->deleteAllMarkers();
        Eigen::Isometry3d ee_pose;
        ee_pose.setIdentity();
        ee_pose.translation() = Eigen::Vector3d(request->target_pose.position.x,request->target_pose.position.y,request->target_pose.position.z);   
        ee_pose.linear() = Eigen::Quaterniond(request->target_pose.orientation.w,request->target_pose.orientation.x,request->target_pose.orientation.y,request->target_pose.orientation.z).toRotationMatrix();  
        response->set__result(visual_tools_->publishEEMarkers(ee_pose,visual_tools_->getRobotModel()->getJointModelGroup("softclaw"),rvt::ORANGE));
    }



    void ExtrinsicCalibrationTaskGenerator::init_visual_tools()
    {
        std::vector<double> table_dimensions;
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(this->shared_from_this()));
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();

        this->get_parameter("table_dimensions",table_dimensions);   
        if(table_dimensions.empty())
        {
            RCLCPP_WARN(this->get_logger(),"Table dimensions is empty, using default");
            table_dimensions = default_table_dimensions;
        }
        else
        {
            if( table_dimensions.size() != 3 && std::any_of(table_dimensions.cbegin(),table_dimensions.cend(),[](double i){return i<=0;}))
            {   
                RCLCPP_WARN(this->get_logger(),"Table dimensions is not set correctly, using default");
                table_dimensions = default_table_dimensions;
            }
        }
        set_up_planning_scene(table_dimensions);
    }

    void ExtrinsicCalibrationTaskGenerator::get_parameters()
    {
        
        this->get_parameter("group_name",group_name_);
        this->get_parameter("eef_ik",eef_ik_);
        this->get_parameter("max_plan_solution",max_plan_solution_);
        this->get_parameter("planning_timeout",planning_timeout_);
        this->get_parameter("visualize_cartesian_path",visualize_cartesian_path_);

    };

    void ExtrinsicCalibrationTaskGenerator::set_up_planning_scene(std::vector<double> table_dimensions)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        //create moveit collision msg 
        moveit_msgs::msg::CollisionObject table_collision;
        table_collision.header.frame_id = "world";
        table_collision.id = "table";
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions.resize(3);
        prim.dimensions[prim.BOX_X] = table_dimensions[0];
        prim.dimensions[prim.BOX_Y] = table_dimensions[1];
        prim.dimensions[prim.BOX_Z] = table_dimensions[2];
        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.z = -table_dimensions[2]/2;
        table_collision.primitives.push_back(prim);
        table_collision.primitive_poses.push_back(table_pose);
        table_collision.operation = table_collision.ADD;
        psi.applyCollisionObject(table_collision);
    }

    bool ExtrinsicCalibrationTaskGenerator::BuildGroupStateTask(std::string group_state_name)
    {
        RCLCPP_INFO(this->get_logger(),"Try to Build Task");
        
        task_.reset(new mtc::Task());
        task_->loadRobotModel(this->shared_from_this());
        // set task properties, moveit group used to plan and ik frame used to plan
        task_->setProperty("group",group_name_);
        RCLCPP_INFO(this->get_logger(),"Task Properties Set");
        // create planner, chose pipeline planner
        RCLCPP_INFO(this->get_logger(),"create planner pipeline");
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
        
        task_->addTaskCallback([this](const mtc::Task& t) { this->ShowErrors(t); });
        RCLCPP_INFO(this->get_logger(),"Planner Created");
        // create a current state stage to generate the inital joint position
        {
            auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
            task_->add(std::move(current_state));
        }
        // create the move to stage setup with target pose and motion duration 
        {
            auto move_to_stage = std::make_unique<mtc::stages::MoveTo>("move_to_pose",sampling_planner);
            // set stage properties from task properties
            RCLCPP_INFO(this->get_logger(),"Set Stage Properties");
            move_to_stage->setGroup(group_name_);
            
            // set goal pose
            RCLCPP_INFO(this->get_logger(),"Set Goal Pose");
            move_to_stage->setGoal(group_state_name);
            move_to_stage->setTimeout(planning_timeout_);

            task_->add(std::move(move_to_stage));

        }
        // prepare task for planning
        try 
        {
            task_->init();
        } 
        catch (mtc::InitStageException& e) 
        {
            RCLCPP_ERROR(this->get_logger(),"Initialization failed: %s",e.what());
            return false;
        }
        RCLCPP_INFO(this->get_logger(),"Task Built");

        return true;
    };

    void ExtrinsicCalibrationTaskGenerator::GroupStatePlan(GroupStateServiceReq::SharedPtr request, GroupStateServiceRes::SharedPtr response)
    {
        bool res;
        MoveitError err;
        if(!task_)
        {
            std::vector<std::string> group_state_names = visual_tools_->getRobotModel()->getJointModelGroup(group_name_)->getDefaultStateNames();
            // RCLCPP_INFO(this->get_logger(),"The default state name are:");
            // for(auto name : group_state_names)
            //     RCLCPP_INFO(this->get_logger(),"-> %s",name.c_str());
            if( std::find(group_state_names.begin(),group_state_names.end(),request->group_state_name) != group_state_names.end())
            {
                res = this->BuildGroupStateTask(request->group_state_name);
                if(res)
                {
                    RCLCPP_INFO(this->get_logger(),"Task Built, Try to Plan");
                     res = this->PlanTask(err);
                    solution_set_ = res;
                    response->set__result(res);
                    response->set__error("Moveit Response: " + moveit::core::error_code_to_string(err));
                }
                else
                {
                    response->set__result(false);
                    response->set__error("Moveit Task Constructor initialization Error");
                }
            }
            else
            {
                response->set__result(false);
                response->set__error("The group name " + request->group_state_name + " has not been found");
            }
        }
        else
        {
            response->set__result(false);
            response->set__error("Another Task has to be executed");
        }
    };

    void ExtrinsicCalibrationTaskGenerator::ExecuteTask(ExecTaskReq::SharedPtr , ExecTaskRes::SharedPtr response)
    {
        MoveitError error;
        if(!solution_set_)
        {   //check if exists solution
            response->set__success(false);
            response->set__message("The Solution Trajectory has not been set");
        }
        else
        {
            //execute the planned solution 
            
            error = task_->execute(*task_->solutions().front());

            if(error == moveit::core::MoveItErrorCode::SUCCESS)
                response->set__success(true);
            else
                response->set__success(false);
            response->set__message("Moveit Response: " + moveit::core::error_code_to_string(error));
            // reset task and disable solution aviability 
            solution_set_ = false;
            task_.reset();
        }
    };
    
    void ExtrinsicCalibrationTaskGenerator::MoveToPlan(
        std::shared_ptr<PlanningReq> request,
        std::shared_ptr<PlanningRes> response 
    )
    {
        bool res;
        MoveitError err;
        TargetPose target;
        if(!task_)
        {
            target.set__pose(request->target_pose);
            target.header.set__frame_id(INERTIAL_FRAME);
            target.header.set__stamp(this->get_clock()->now());
            RCLCPP_INFO(this->get_logger(),"Received request to move to target pose");
            res = this->BuildGoToTask(target);
            if(res)
            {
                RCLCPP_INFO(this->get_logger(),"Task Built, Try to Plan");
                res = this->PlanTask(err);
                solution_set_ = res;
                response->set__result(res);
                response->set__error("Moveit Response: " + moveit::core::error_code_to_string(err));
            }
            else
            {
                response->set__result(false);
                response->set__error("Moveit Task Constructor initialization Error");
            }
        }
        else
        {
            response->set__result(false);
            response->set__error("Another Task has to be executed");
        }
    };

    bool ExtrinsicCalibrationTaskGenerator::BuildGoToTask(
        TargetPose target_pose
    )
    {
        RCLCPP_INFO(this->get_logger(),"Try to Build Task");
        
        task_.reset(new mtc::Task());
        task_->loadRobotModel(this->shared_from_this());
        // set task properties, moveit group used to plan and ik frame used to plan
        task_->setProperty("group",group_name_);
        task_->setProperty("ik_frame",eef_ik_);
        RCLCPP_INFO(this->get_logger(),"Task Properties Set");
        // create planner, chose pipeline planner
        RCLCPP_INFO(this->get_logger(),"create planner pipeline");
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
        
        task_->addTaskCallback([this](const mtc::Task& t) { this->ShowErrors(t); });
          
        // set goal pose tollerance TODO set as parameter
        RCLCPP_INFO(this->get_logger(),"set tollerance");
        sampling_planner->setProperty("goal_position_tolerance",1e-3);
        RCLCPP_INFO(this->get_logger(),"position tollerance created");
        sampling_planner->setProperty("goal_orientation_tolerance",1e-2);
        RCLCPP_INFO(this->get_logger(),"Planner Created");
        // trajectory_processing::TimeParameterizationPtr tp() = std::make_shared<trajectory_processing::TimeParameterization>();
        // sampling_planner->setTimeParameterization()
        // create a current state stage to generate the inital joint position
        {
            auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
            task_->add(std::move(current_state));
        }
        // create the move to stage setup with target pose and motion duration 
        {
            auto move_to_stage = std::make_unique<mtc::stages::MoveTo>("move_to_pose",sampling_planner);
            // set stage properties from task properties
            RCLCPP_INFO(this->get_logger(),"Set Stage Properties");
            move_to_stage->setGroup(group_name_);
            RCLCPP_INFO(this->get_logger(),"Set Stage Properties");
            move_to_stage->setIKFrame(eef_ik_);
            // set goal pose
            RCLCPP_INFO(this->get_logger(),"Set Goal Pose");
            move_to_stage->setGoal(target_pose);
            move_to_stage->setTimeout(planning_timeout_);

            task_->add(std::move(move_to_stage));

        }
        // prepare task for planning
        try 
        {
            task_->init();
        } 
        catch (mtc::InitStageException& e) 
        {
            RCLCPP_ERROR(this->get_logger(),"Initialization failed: %s",e.what());
            return false;
        }
        RCLCPP_INFO(this->get_logger(),"Task Built");
        return true;
    };

    bool ExtrinsicCalibrationTaskGenerator::PlanTask(MoveitError& error_code)
    {
        SolutionServiceRequest::SharedPtr sol_req = std::make_shared<SolutionServiceRequest>();
        SolutionServiceResponse::SharedPtr sol_res = std::make_shared<SolutionServiceResponse>();
        RCLCPP_INFO(this->get_logger(),"Start searching for task solutions");
        // plan the task
        error_code = task_->plan(static_cast<std::size_t>(this->get_parameter("max_plan_solution").as_int()));
        
        if(error_code)
        {
            
            RCLCPP_INFO(this->get_logger(),"Planning succeded");  
            if(visualize_cartesian_path_)
            {
                visual_tools_->deleteAllMarkers();
                task_->publishAllSolutions(false);
                sol_req->set__solution_id(task_->introspection().solutionId(*task_->solutions().front()));
                task_->introspection().getSolution(sol_req,sol_res);
                for(auto & traj : sol_res->solution.sub_trajectory)
                {
                    visual_tools_->publishTrajectoryLine(traj.trajectory,
                    visual_tools_->getRobotModel()->getJointModelGroup("ur_softclaw_manipulator"));
                }
                visual_tools_->trigger();
                RCLCPP_INFO(this->get_logger(),"Visualize Cartesian Path");     
            }
            // exec_solution_.reset(task_->solutions().front().get());

        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Planning failed");
            return false;
        }
        return true;
        
    }
};

//     ExtrinsicCalibrationMoveGroupInterface::ExtrinsicCalibrationMoveGroupInterface(const rclcpp::NodeOptions& options):
//     Node("the_mgi_calibrator",options)
//     {
//         // declare planning parameter
//         if(!this->has_parameter("group_name"))
//             declare_parameter<std::string>("group_name","ur_manipulator");
//         if(!this->has_parameter("eef_ik"))
//             declare_parameter<std::string>("eef_ik","tool0");
//         if(!this->has_parameter("max_plan_solution"))
//             declare_parameter<int>("max_plan_solution",2);
//         if(!this->has_parameter("planning_timeout"))
//             declare_parameter<double>("planning_timeout",30.0);
//         if(!this->has_parameter("visualize_cartesian_path"))
//             declare_parameter<bool>("visualize_cartesian_path",true);
//         if(!this->has_parameter("gui_debug"))
//             declare_parameter<bool>("gui_debug",true);

//         if(!this->has_parameter("table_dimensions"))
//             declare_parameter<std::vector<double>>("table_dimensions",std::vector<double>());
        
        

//         // create service for motion
//         motion_service_ = this->create_service<MotionService>(
//             "~/set_goal",
//             std::bind(&ExtrinsicCalibrationMoveGroupInterface::MoveToPlanExec,this,_1,_2)
//         );
//         // create service for visualize end effector
//         visual_end_eff_service_ = this->create_service<VisualEndEffService>(
//             "~/visual_end_eff",
//             std::bind(&ExtrinsicCalibrationMoveGroupInterface::VisualEndEff,this,_1,_2)

//         );

//         // prova_ = this->create_subscription<sensor_msgs::msg::JointState>(
//         //     "joint_states",
//         //     10,
//         //     std::bind(&ExtrinsicCalibrationMoveGroupInterface::prova_call,this,_1)
//         //     );
//         RCLCPP_INFO(this->get_logger(),"Extrinsic Calibration Task Generator Node Created");
//     }

//     void ExtrinsicCalibrationMoveGroupInterface::get_parameters()
//     {
        
//         this->get_parameter("group_name",group_name_);
//         this->get_parameter("eef_ik",eef_ik_);
//         this->get_parameter("max_plan_solution",max_plan_solution_);
//         this->get_parameter("planning_timeout",planning_timeout_);
//         this->get_parameter("visualize_cartesian_path",visualize_cartesian_path_);
//         this->get_parameter("gui_debug",gui_debug_);

//     };

//     void ExtrinsicCalibrationMoveGroupInterface::init_visual_tools()
//     {
//         std::vector<double> table_dimensions;
//         visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(this->shared_from_this()));
//         visual_tools_->deleteAllMarkers();
//         if(gui_debug_)
//             visual_tools_->loadRemoteControl();
//         visual_tools_->trigger();

//         this->get_parameter("table_dimensions",table_dimensions);   
//         if(table_dimensions.empty())
//         {
//             RCLCPP_WARN(this->get_logger(),"Table dimensions is empty, using default");
//             table_dimensions = default_table_dimensions;
//         }
//         else
//         {
//             if( table_dimensions.size() != 3 && std::any_of(table_dimensions.cbegin(),table_dimensions.cend(),[](double i){return i<=0;}))
//             {   
//                 RCLCPP_WARN(this->get_logger(),"Table dimensions is not set correctly, using default");
//                 table_dimensions = default_table_dimensions;
//             }
//         }
//         set_up_planning_scene(table_dimensions);
        
//     }

//     void ExtrinsicCalibrationMoveGroupInterface::set_up_planning_scene(std::vector<double> table_dimensions)
//     {
//         moveit::planning_interface::PlanningSceneInterface psi;
//         //create moveit collision msg 
//         moveit_msgs::msg::CollisionObject table_collision;
//         table_collision.header.frame_id = "world";
//         table_collision.id = "table";
//         shape_msgs::msg::SolidPrimitive prim;
//         prim.type = prim.BOX;
//         prim.dimensions.resize(3);
//         prim.dimensions[prim.BOX_X] = table_dimensions[0];
//         prim.dimensions[prim.BOX_Y] = table_dimensions[1];
//         prim.dimensions[prim.BOX_Z] = table_dimensions[2];
//         geometry_msgs::msg::Pose table_pose;
//         table_pose.orientation.w = 1.0;
//         table_pose.position.z = -table_dimensions[2]/2;
//         table_collision.primitives.push_back(prim);
//         table_collision.primitive_poses.push_back(table_pose);
//         table_collision.operation = table_collision.ADD;
//         psi.applyCollisionObject(table_collision);
//     }

//     void ExtrinsicCalibrationMoveGroupInterface::create_mg_interface()
//     {
//         this->get_parameters();
//         move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(this->shared_from_this(),group_name_));
//         move_group_interface_->startStateMonitor(5.0);
//     }

//     void ExtrinsicCalibrationMoveGroupInterface::VisualEndEff(std::shared_ptr<VisualEndEffReq> request, std::shared_ptr<VisualEndEffRes> response)
//     {
//         RCLCPP_INFO(this->get_logger(),"Received request to visualize end effector");
//         visual_tools_->deleteAllMarkers();
//         Eigen::Isometry3d ee_pose;
//         ee_pose.setIdentity();
//         ee_pose.translation() = Eigen::Vector3d(request->target_pose.position.x,request->target_pose.position.y,request->target_pose.position.z);   
//         ee_pose.linear() = Eigen::Quaterniond(request->target_pose.orientation.w,request->target_pose.orientation.x,request->target_pose.orientation.y,request->target_pose.orientation.z).toRotationMatrix();  
//         response->set__result(visual_tools_->publishEEMarkers(ee_pose,visual_tools_->getRobotModel()->getJointModelGroup("softclaw"),rvt::ORANGE));
//     };

//     void ExtrinsicCalibrationMoveGroupInterface::MoveToPlanExec(std::shared_ptr<MotionReq> request, std::shared_ptr<MotionRes> response)
//     {
//         moveit::planning_interface::MoveGroupInterface::Plan path_planned;
//         moveit::core::MoveItErrorCode result;
//         RCLCPP_INFO(this->get_logger(),"Called move to service");
//         move_group_interface_->setPoseTarget(request->target_pose,eef_ik_);
//         result = move_group_interface_->plan(path_planned);
//         if(result == moveit::core::MoveItErrorCode::SUCCESS)
//         {
//             if(visualize_cartesian_path_)
//             {
//                 visual_tools_->publishAxisLabeled(request->target_pose,"target_pose");
//                 visual_tools_->publishTrajectoryLine(path_planned.trajectory_,visual_tools_->getRobotModel()->getJointModelGroup(group_name_));
//                 visual_tools_->trigger();
                
//             }
//             // move_group_interface_->execute(path_planned);
//             if(gui_debug_)
//                 visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to execute the task ");
//             result =  move_group_interface_->execute(path_planned);
//             if(result == moveit::core::MoveItErrorCode::SUCCESS)
//                 response->set__result(true);
//             else 
//                 response->set__result(false);
//             response->set__error(moveit::core::error_code_to_string(result));
            
//         }
//     };

// };