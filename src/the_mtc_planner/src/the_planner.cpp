#include "the_mtc_planner/the_planner.hpp"
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/utils/moveit_error_code.h>
#include <string>
#include <memory>
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>


namespace the_task_generator
{

    using std::placeholders::_1;
    using std::placeholders::_2;
    using Stage = mtc::Stage;
    using SolutionService = moveit_task_constructor_msgs::srv::GetSolution;
    using SolutionServiceRequest = SolutionService::Request;
    using SolutionServiceResponse = SolutionService::Response;

    TaskConstructorPlanner::TaskConstructorPlanner(const rclcpp::NodeOptions& options):
    rclcpp::Node("the_calibration_planner", options)
    {
        declare_parameters();
        
        get_parameters();

        
        // create service for motion
        motion_service_ = this->create_service<PlanningService>(
            "~/target_pose_plan",
            std::bind(&TaskConstructorPlanner::MoveToPlan,this,_1,_2)
        );

        execute_task_service_ = this->create_service<ExecTask>(
            "~/exec_planned_task",
            std::bind(&TaskConstructorPlanner::ExecuteTask,this,_1,_2)
        );

        // create service for visualize end effector
        visual_end_eff_service_ = this->create_service<VisualEndEffService>(
            "~/visual_end_eff",
            std::bind(&TaskConstructorPlanner::VisualEndEff,this,_1,_2)
        );

        group_motion_service_ = this->create_service<GroupStateService>(
            "~/group_state_plan",
            std::bind(&TaskConstructorPlanner::GroupStatePlan,this,_1,_2)
        );

        // create service for worksapce update
        move_ws_service_ = this->create_service<MoveWS>(
            "~/move_workspace",
            std::bind(&TaskConstructorPlanner::move_ws,this,_1,_2)
        );

        RCLCPP_INFO(this->get_logger(),"Task Constructor Planner Node Created");

    };

    void TaskConstructorPlanner::declare_parameters()
    {
        if(!this->has_parameter("group_name"))
            declare_parameter<std::string>("group_name","ur_manipulator");
        if(!this->has_parameter("default_eef_ik"))
            declare_parameter<std::string>("default_eef_ik","fixed_fingertip");
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
        if(!this->has_parameter("workspace_dimensions_camera_frame"))
            declare_parameter<std::vector<double>>("workspace_dimensions_camera_frame",std::vector<double>({0.5,0.5,0.5}));

    };

    void TaskConstructorPlanner::get_parameters()
    {
        double quaternion_norm;
        std::vector<double> table_dimensions, camera_frame_position, camera_frame_orientation, workspace_dimensions_camera_frame;
        this->get_parameter("group_name",group_name_);
        this->get_parameter("default_eef_ik",default_eef_ik_);
        this->get_parameter("max_plan_solution",max_plan_solution_);
        this->get_parameter("planning_timeout",planning_timeout_);
        this->get_parameter("visualize_cartesian_path",visualize_cartesian_path_);

        this->get_parameter("camera_frame_position",camera_frame_position);
        this->get_parameter("camera_frame_orientation",camera_frame_orientation);

        if(camera_frame_position.size() != 3 || camera_frame_orientation.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(),"Camera frame position or orientation has not the correct size");
            throw std::runtime_error("Camera frame position or orientation has not the correct size");
        }
        quaternion_norm = std::sqrt(camera_frame_orientation[0]*camera_frame_orientation[0] + camera_frame_orientation[1]*camera_frame_orientation[1] + camera_frame_orientation[2]*camera_frame_orientation[2] + camera_frame_orientation[3]*camera_frame_orientation[3]);
        if(std::abs(quaternion_norm - 1.0) > 1e-4)
        {
            RCLCPP_ERROR(this->get_logger(),"Camera frame orientation is not a unit quaternion");
            throw std::runtime_error("Camera frame orientation is not a unit quaternion");
        }
        
        Vector3d eigen_pos = Vector3d(camera_frame_position[0],camera_frame_position[1],camera_frame_position[2]); 
        Quaterniond eigen_ori = Quaterniond(camera_frame_orientation[0],camera_frame_orientation[1],camera_frame_orientation[2],camera_frame_orientation[3]);

        camera_frame_pose_ = Affine3d();
        camera_frame_pose_.translation() = eigen_pos;
        camera_frame_pose_.linear() = eigen_ori.toRotationMatrix();
        };

    void TaskConstructorPlanner::init_visual_tools()
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

    void TaskConstructorPlanner::set_up_planning_scene(std::vector<double> table_dimensions)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        //create moveit collision msg 
        moveit_msgs::msg::CollisionObject table_collision,workspace_collision;
        table_collision.header.frame_id = "world";
        table_collision.id = TABLE_NAME;
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
        add_workspace_collision(workspace_collision);
        psi.applyCollisionObject(workspace_collision);



    }

    void TaskConstructorPlanner::add_workspace_collision(moveit_msgs::msg::CollisionObject& msg)
    {
        std::vector<double> workspace_dimensions;
        Vector3d world_pos,camera_pos;
        Quaterniond camera_ori;
        this->get_parameter("workspace_dimensions_camera_frame",workspace_dimensions);
        if(workspace_dimensions.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(),"Workspace dimensions has not the correct size, use default");
            workspace_dimensions = WORKSPACE_DIMENSIONS;
        }

        msg.header.frame_id = INERTIAL_FRAME;
        msg.id = WORKSPACE_NAME;
        shape_msgs::msg::SolidPrimitive left_wall,right_wall, back_wall, up_wall;
        geometry_msgs::msg::Pose right_wall_pose,left_wall_pose, back_wall_pose, up_wall_pose;
        // right wall dimension and pose
        right_wall.type = right_wall.BOX;
        right_wall.dimensions.resize(3);
        right_wall.dimensions[right_wall.BOX_X] = camera_frame_pose_.translation()(2);
        right_wall.dimensions[right_wall.BOX_Y] = OBSTACLE_THICKNESS;
        right_wall.dimensions[right_wall.BOX_Z] = workspace_dimensions[2];
        camera_pos << camera_frame_pose_.translation()(2)/2, -workspace_dimensions[0]-OBSTACLE_THICKNESS/2,-workspace_dimensions[2]/2;
        world_pos = camera_frame_pose_ * camera_pos;
        right_wall_pose.position.x = world_pos(0);
        right_wall_pose.position.y = world_pos(1);
        right_wall_pose.position.z = world_pos(2);
        camera_ori = Quaterniond(camera_frame_pose_.rotation());
        right_wall_pose.orientation.w = camera_ori.w();
        right_wall_pose.orientation.x = camera_ori.x();
        right_wall_pose.orientation.y = camera_ori.y();
        right_wall_pose.orientation.z = camera_ori.z();

        msg.primitives.push_back(right_wall);
        msg.primitive_poses.push_back(right_wall_pose);
       //left wall dimension and pose
        left_wall.type = left_wall.BOX;
        left_wall.dimensions.resize(3);
        left_wall.dimensions[left_wall.BOX_X] = camera_frame_pose_.translation()(2);
        left_wall.dimensions[left_wall.BOX_Y] = OBSTACLE_THICKNESS;
        left_wall.dimensions[left_wall.BOX_Z] = workspace_dimensions[2];
        camera_pos << camera_frame_pose_.translation()(2)/2, workspace_dimensions[1]+OBSTACLE_THICKNESS/2,-workspace_dimensions[2]/2;
        world_pos = camera_frame_pose_ * camera_pos;
        left_wall_pose.position.x = world_pos(0);
        left_wall_pose.position.y = world_pos(1);
        left_wall_pose.position.z = world_pos(2);
        camera_ori = Quaterniond(camera_frame_pose_.rotation());
        left_wall_pose.orientation.w = camera_ori.w();
        left_wall_pose.orientation.x = camera_ori.x();
        left_wall_pose.orientation.y = camera_ori.y();
        left_wall_pose.orientation.z = camera_ori.z();

        msg.primitives.push_back(left_wall);
        msg.primitive_poses.push_back(left_wall_pose);

        //back wall dimension and pose
        back_wall.type = back_wall.BOX;
        back_wall.dimensions.resize(3); 
        back_wall.dimensions[back_wall.BOX_X] = camera_frame_pose_.translation()(2);
        back_wall.dimensions[back_wall.BOX_Y] = workspace_dimensions[1] + workspace_dimensions[0];
        back_wall.dimensions[back_wall.BOX_Z] = OBSTACLE_THICKNESS;
        camera_pos << camera_frame_pose_.translation()(2)/2, (workspace_dimensions[1] - workspace_dimensions[0])/2,-workspace_dimensions[2]-OBSTACLE_THICKNESS/2;
        world_pos = camera_frame_pose_ * camera_pos;
        back_wall_pose.position.x = world_pos(0);
        back_wall_pose.position.y = world_pos(1);
        back_wall_pose.position.z = world_pos(2);
        camera_ori = Quaterniond(camera_frame_pose_.rotation());
        back_wall_pose.orientation.w = camera_ori.w();
        back_wall_pose.orientation.x = camera_ori.x();
        back_wall_pose.orientation.y = camera_ori.y();
        back_wall_pose.orientation.z = camera_ori.z();

        msg.primitives.push_back(back_wall);
        msg.primitive_poses.push_back(back_wall_pose);

        //up wall dimension and pose
        up_wall.type = up_wall.BOX;
        up_wall.dimensions.resize(3);
        up_wall.dimensions[up_wall.BOX_X] = OBSTACLE_THICKNESS;
        up_wall.dimensions[up_wall.BOX_Y] = workspace_dimensions[1] + workspace_dimensions[0];
        up_wall.dimensions[up_wall.BOX_Z] = workspace_dimensions[2];
        camera_pos << -OBSTACLE_THICKNESS/2,(workspace_dimensions[1] - workspace_dimensions[0])/2,-workspace_dimensions[2]/2;
        world_pos = camera_frame_pose_ * camera_pos;
        up_wall_pose.position.x = world_pos(0);
        up_wall_pose.position.y = world_pos(1);
        up_wall_pose.position.z = world_pos(2);
        camera_ori = Quaterniond(camera_frame_pose_.rotation());
        up_wall_pose.orientation.w = camera_ori.w();
        up_wall_pose.orientation.x = camera_ori.x();
        up_wall_pose.orientation.y = camera_ori.y();
        up_wall_pose.orientation.z = camera_ori.z();

        msg.primitives.push_back(up_wall);
        msg.primitive_poses.push_back(up_wall_pose);

        msg.operation = msg.ADD; 
    }

    void TaskConstructorPlanner::move_ws(MoveWSReq::SharedPtr request, MoveWSRes::SharedPtr response)
    {
        bool res;
        Vector3d camera_pos;
        Quaterniond camera_ori;
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject workspace_collision;

        workspace_collision.header.frame_id = INERTIAL_FRAME;
        workspace_collision.id = WORKSPACE_NAME;
        workspace_collision.operation = workspace_collision.REMOVE;
        if(!psi.applyCollisionObject(workspace_collision))
        {
            response->set__result(false);
            return;
        }
        // update  local variable
        camera_pos << request->target_pose.position.x,request->target_pose.position.y,request->target_pose.position.z;
        camera_ori = Quaterniond(request->target_pose.orientation.w,request->target_pose.orientation.x,request->target_pose.orientation.y,request->target_pose.orientation.z);
        camera_frame_pose_.translation() = camera_pos;
        camera_frame_pose_.linear() = camera_ori.toRotationMatrix();
        add_workspace_collision(workspace_collision);
        if(psi.applyCollisionObject(workspace_collision))
            response->set__result(true);
        else
            response->set__result(false);

    };

    void TaskConstructorPlanner::VisualEndEff(
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

     bool TaskConstructorPlanner::BuildGroupStateTask(std::string group_state_name)
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

    void TaskConstructorPlanner::GroupStatePlan(GroupStateServiceReq::SharedPtr request, GroupStateServiceRes::SharedPtr response)
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

    void TaskConstructorPlanner::ExecuteTask(ExecTaskReq::SharedPtr , ExecTaskRes::SharedPtr response)
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
    
    void TaskConstructorPlanner::MoveToPlan(
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

    bool TaskConstructorPlanner::BuildGoToTask(
        TargetPose target_pose
    )
    {
        RCLCPP_INFO(this->get_logger(),"Try to Build Task");
        
        task_.reset(new mtc::Task());
        task_->loadRobotModel(this->shared_from_this());
        // set task properties, moveit group used to plan and ik frame used to plan
        task_->setProperty("group",group_name_);
        task_->setProperty("ik_frame",default_eef_ik_);
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
            move_to_stage->setIKFrame(default_eef_ik_);
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

    bool TaskConstructorPlanner::PlanTask(MoveitError& error_code)
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
        
    };
    void TaskConstructorPlanner::ShowErrors(const mtc::Task& t)
    {
        std::ostringstream os;
        t.printState(os);
        t.explainFailure(os);
        RCLCPP_INFO(this->get_logger(), "%s", os.str().c_str());
        
    };

};