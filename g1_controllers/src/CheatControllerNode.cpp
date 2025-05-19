//
// Created by pocket on 24-2-11.
//

#include "g1_controllers/g1Controller.h"
#include <thread>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

bool pause_flag = false;

void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg){
    pause_flag = msg->data;
    std::cerr << "pause_flag: " << pause_flag << std::endl;
}

int main(int argc, char** argv){
    rclcpp::Duration elapsedTime_ = rclcpp::Duration::from_seconds(0.002);


    // Initialize ros node    
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
    "g1_controller_node",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

    //create a subscriber to pauseFlag
    //这个订阅者监听名为"pauseFlag"的话题（topic），该话题发布布尔类型的消息。
    //当收到消息时，会调用pauseCallback回调函数处理这些消息, 只保留最新的一条消息. 
    auto pause_sub = node->create_subscription<std_msgs::msg::Bool>("pauseFlag", 1, pauseCallback);
    g1_controller::g1CheaterController controller;
    if (!controller.init(node)) {
        RCLCPP_ERROR(node->get_logger(),"Failed to initialize the g1 controller!");
        return -1;
    }

    auto startTime = Clock::now(); //系统高精度时钟,精确测量控制循环的时间间隔 
    auto startTimeROS = node->get_clock()->now(); //保持与ROS系统同步
    controller.starting(startTimeROS);
    auto lastTime = startTime;

    //create a thread to spin the node
    std::thread spin_thread([node](){
        rclcpp::spin(node);
    });
    spin_thread.detach();

    while(rclcpp::ok()){
        if (!pause_flag)
        {
            const auto currentTime = Clock::now();
            // Compute desired duration rounded to clock decimation
            const Duration desiredDuration(1.0 / 500);

            // Get change in time
            Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime);
            elapsedTime_ = rclcpp::Duration::from_seconds(time_span.count());
            lastTime = currentTime;

            // Check cycle time for excess delay
//            const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
//            if (cycle_time_error > cycleTimeErrorThreshold_) {
//                ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
//                                                                           << "cycle time: " << elapsedTime_ << "s, "
//                                                                           << "threshold: " << cycleTimeErrorThreshold_ << "s");
//            }

            // Control
            // let the controller compute the new command (via the controller manager)
            controller.update(node->get_clock()->now(), elapsedTime_);

            // Sleep
            const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
            std::this_thread::sleep_until(sleepTill);
        }
    }



    return 0;
}