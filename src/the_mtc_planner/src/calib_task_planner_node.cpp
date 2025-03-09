// #include "the_mtc_planner/calib_task_planner.hpp"
#include "the_mtc_planner/the_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main( int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<the_task_generator::TaskConstructorPlanner>(options);
    
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });
    node->get_parameters();
    node->init_visual_tools();
    // try
    // {
    //     executor.spin();
    // }
    // catch(std::exception& e)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error in calib_task_planner_no de: %s", e.what());
    //      rclcpp::shutdown();
    // }
    if(spinner_thread.joinable())
        spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}