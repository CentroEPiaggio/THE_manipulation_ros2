#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>
// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "ur_softclaw_manipulator";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.hpp>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.hpp>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  move_group.setPlannerId("RRTstarkConfigDefault");
  // add collision plane 
  moveit_msgs::msg::CollisionObject table_collision;
  table_collision.header.set__frame_id(move_group.getPlanningFrame());
  table_collision.set__id("table");

  shape_msgs::msg::SolidPrimitive prim;

  prim.set__type(prim.BOX);
  prim.dimensions.resize(3);
  prim.dimensions[prim.BOX_X] = 5.0;
  prim.dimensions[prim.BOX_Y] = 5.0;
  prim.dimensions[prim.BOX_Z] = 0.2;

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.set__w(1.0);
  table_pose.position.set__x(0.0);
  table_pose.position.set__y(0.0);
  table_pose.position.set__z(-0.11);

  table_collision.primitives.push_back(prim);
  table_collision.primitive_poses.push_back(table_pose);
  table_collision.operation = table_collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(table_collision);

  RCLCPP_WARN(LOGGER,"ADD table to the planning scene");
  planning_scene_interface.addCollisionObjects(objects);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  auto ee_names = joint_model_group->getAttachedEndEffectorNames();
  RCLCPP_INFO(LOGGER,"%d",ee_names.size());
  auto link_name_jmg = joint_model_group->getLinkModelNames();
  RCLCPP_WARN(LOGGER,"links");
  for(auto link : link_name_jmg)
    RCLCPP_INFO(LOGGER, "%s",link.c_str());
  
  std::vector<std::string> tips;
  if(!joint_model_group->getEndEffectorTips(tips) || tips.empty())
    RCLCPP_ERROR(LOGGER,"DIO CARO no");
  else
    RCLCPP_ERROR(LOGGER,"DIO CARO si %s",tips[0].c_str());  // Visualization
  if(tips.empty())
    RCLCPP_ERROR(LOGGER,"DIOCARO OOOOOOO");
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("ur_manipulator");
  std::pair<std::string,std::string> name_pair = joint_model_group->getEndEffectorParentGroup();
  RCLCPP_ERROR(LOGGER,"DIOCARO %s and sec %s",name_pair.first.c_str(),name_pair.second.c_str());

  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("ur_softclaw_manipulator");
  name_pair = joint_model_group->getEndEffectorParentGroup();
  RCLCPP_ERROR(LOGGER,"DIOCARO %s and sec %s",name_pair.first.c_str(),name_pair.second.c_str());

  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("gripper");
  name_pair = joint_model_group->getEndEffectorParentGroup();
  RCLCPP_ERROR(LOGGER,"DIOCARO %s and sec %s",name_pair.first.c_str(),name_pair.second.c_str());
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world", "test_mg",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  auto robot_model_ptr = move_group.getRobotModel();

  moveit::core::RobotStatePtr robot_stt_ptr(new moveit::core::RobotState(robot_model_ptr));

  Eigen::Isometry3d ee_pose = Eigen::Isometry3d::Identity();
  Eigen::Quaternion rot = Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(-3.140, Eigen::Vector3d::UnitX());
  Eigen::Vector3d pos(-0.032,  0.790, 0.439);
  ee_pose.linear() = rot.toRotationMatrix();
  ee_pose.translation() = pos;

  if(!robot_stt_ptr->setFromIK(robot_model_ptr->getJointModelGroup("ur_manipulator"), ee_pose, 0.1))
  {
    RCLCPP_ERROR(LOGGER,"IK FAILED set to random positions");
    robot_stt_ptr->setToRandomPositions();
  }
  else
  { 
      std::vector<double> joint_values;
      RCLCPP_INFO(LOGGER,"IK SUCCESS");
      robot_stt_ptr->copyJointGroupPositions("ur_manipulator", joint_values);
      robot_stt_ptr->setJointGroupPositions("ur_manipulator", joint_values);


  } 


  

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.publishRobotState(robot_stt_ptr, rvt::ORANGE);
  
  visual_tools.publishEEMarkers(ee_pose, move_group.getCurrentState()->getJointModelGroup("gripper") , rvt::RED,"tip");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups they are %d:", move_group.getJointModelGroupNames().size());
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  auto links = move_group.getLinkNames();
  RCLCPP_INFO(LOGGER,"Get link names");
  for(auto link : links)
    RCLCPP_INFO(LOGGER,"%s",link.c_str());
  
  
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 0.880;
  target_pose1.orientation.y = 0.458;
  target_pose1.orientation.z = 0.115;
  target_pose1.orientation.w = -0.061;
  
  target_pose1.position.x = 0.218;
  target_pose1.position.y = 1.048;
  target_pose1.position.z = 0.964;
  move_group.setPoseTarget(target_pose1);
  // move_group.setPlanningTime(60);
  // move_group.setPlanningTime(6000);
  // move_group.setReplanAttempts(15);
  // move_group.setPositionTarget(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  auto err = move_group.execute(my_plan.trajectory_);
  RCLCPP_ERROR(LOGGER,"%d",err);

  // // Moving to a pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Moving to a pose goal is similar to the step above
  // // except we now use the ``move()`` function. Note that
  // // the pose goal we had set earlier is still active
  // // and so the robot will try to move to that goal. We will
  // // not use that function in this tutorial since it is
  // // a blocking function and requires a controller to be active
  // // and report success on execution of a trajectory.

  // /* Uncomment below line when working with a real robot */
  // /* move_group.move(); */

  // // Planning to a joint-space goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Let's set a joint space goal and move towards it.  This will replace the
  // // pose target we set above.
  // //
  // // To start, we'll create an pointer that references the current robot's state.
  // // RobotState is the object that contains all the current position/velocity/acceleration data.
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  // //
  // // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  // joint_group_positions[0] = -1.0;  // radians
  // bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  // }

  // // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // // The default values are 10% (0.1).
  // // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // // or set explicit factors in your code if you need your robot to move faster.
  // move_group.setMaxVelocityScalingFactor(0.05);
  // move_group.setMaxAccelerationScalingFactor(0.05);

  // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz:
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Planning with Path Constraints
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Path constraints can easily be specified for a link on the robot.
  // // Let's specify a path constraint and a pose goal for our group.
  // // First define the path constraint.
  // moveit_msgs::msg::OrientationConstraint ocm;
  // ocm.link_name = "panda_link7";
  // ocm.header.frame_id = "panda_link0";
  // ocm.orientation.w = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::msg::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(test_constraints);

  // // Enforce Planning in Joint Space
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Depending on the planning problem MoveIt chooses between
  // // ``joint space`` and ``cartesian space`` for problem representation.
  // // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  // //
  // // By default, planning requests with orientation path constraints
  // // are sampled in ``cartesian space`` so that invoking IK serves as a
  // // generative sampler.
  // //
  // // By enforcing ``joint space``, the planning process will use rejection
  // // sampling to find valid requests. Please note that this might
  // // increase planning time considerably.
  // //
  // // We will reuse the old goal that we had and plan to it.
  // // Note that this will only work if the current state already
  // // satisfies the path constraints. So we need to set the start
  // // state to a new pose.
  // moveit::core::RobotState start_state(*move_group.getCurrentState());
  // geometry_msgs::msg::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  // start_pose2.position.x = 0.55;
  // start_pose2.position.y = -0.05;
  // start_pose2.position.z = 0.8;
  // start_state.setFromIK(joint_model_group, start_pose2);
  // move_group.setStartState(start_state);

  // // Now, we will plan to the earlier pose target from the new
  // // start state that we just created.
  // move_group.setPoseTarget(target_pose1);

  // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  // move_group.setPlanningTime(10.0);

  // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz:
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(start_pose2, "start");
  // visual_tools.publishAxisLabeled(target_pose1, "goal");
  // visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // When done with the path constraint, be sure to clear it.
  // move_group.clearPathConstraints();

  // // Cartesian Paths
  // // ^^^^^^^^^^^^^^^
  // // You can plan a Cartesian path directly by specifying a list of waypoints
  // // for the end-effector to go through. Note that we are starting
  // // from the new start state above.  The initial pose (start state) does not
  // // need to be added to the waypoint list but adding it can help with visualizations
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // waypoints.push_back(start_pose2);

  // geometry_msgs::msg::Pose target_pose3 = start_pose2;

  // target_pose3.position.z -= 0.2;
  // waypoints.push_back(target_pose3);  // down

  // target_pose3.position.y -= 0.2;
  // waypoints.push_back(target_pose3);  // right

  // target_pose3.position.z += 0.2;
  // target_pose3.position.y += 0.2;
  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // up and left

  // // We want the Cartesian path to be interpolated at a resolution of 1 cm,
  // // which is why we will specify 0.01 as the max step in Cartesian translation.
  // const double eef_step = 0.01;
  // moveit_msgs::msg::RobotTrajectory trajectory;
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, 0.5, trajectory);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Cartesian motions should often be slow, e.g. when approaching objects. The speed of Cartesian
  // // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // // Pull requests are welcome.
  // //
  // // You can execute a trajectory like this.
  // /* move_group.execute(trajectory); */

  // // Adding objects to the environment
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // First, let's plan to another simple goal with no objects in the way.
  // move_group.setStartState(*move_group.getCurrentState());
  // geometry_msgs::msg::Pose another_pose;
  // another_pose.orientation.w = 0;
  // another_pose.orientation.x = -1.0;
  // another_pose.position.x = 0.7;
  // another_pose.position.y = 0.0;
  // another_pose.position.z = 0.59;
  // move_group.setPoseTarget(another_pose);

  // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishAxisLabeled(another_pose, "goal");
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // The result may look like this:
  // //
  // // .. image:: ./move_group_interface_tutorial_clear_path.gif
  // //    :alt: animation showing the arm moving relatively straight toward the goal
  // //
  // // Now, let's define a collision object ROS message for the robot to avoid.
  // moveit_msgs::msg::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();

  // // The id of the object is used to identify it.
  // collision_object.id = "box1";

  // // Define a box to add to the world.
  // shape_msgs::msg::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.1;
  // primitive.dimensions[primitive.BOX_Y] = 1.5;
  // primitive.dimensions[primitive.BOX_Z] = 0.5;

  // // Define a pose for the box (specified relative to frame_id).
  // geometry_msgs::msg::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.48;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = 0.25;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  // // Now, let's add the collision object into the world
  // // (using a vector that could contain additional objects)
  // RCLCPP_INFO(LOGGER, "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // // Now, when we plan a trajectory it will avoid the obstacle.
  // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  // visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach an object to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between objects as well.
//   moveit_msgs::msg::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::msg::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper.
//   object_to_attach.header.frame_id = move_group.getEndEffectorLink();
//   geometry_msgs::msg::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector).
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   RCLCPP_INFO(LOGGER, "Attach the object to the robot");
//   std::vector<std::string> touch_links;
//   touch_links.push_back("panda_rightfinger");
//   touch_links.push_back("panda_leftfinger");
//   move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

//   visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group.setStartStateToCurrentState();
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   RCLCPP_INFO(LOGGER, "Detach the object from the robot");
//   move_group.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   RCLCPP_INFO(LOGGER, "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

//   // END_TUTORIAL
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}