#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_server/action/navigate_to_map.hpp"

#include <tf2/LinearMath/Quaternion.h> 

using namespace std::chrono_literals;
using NavigateToMap = nav_server::action::NavigateToMap;
using GoalHandleNavigateToMap = rclcpp_action::ServerGoalHandle<NavigateToMap>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class MultiMapActionServer : public rclcpp::Node {
public:
    MultiMapActionServer()
    : Node("nav_server")
    {
        // Initialize action server
        action_server_ = rclcpp_action::create_server<NavigateToMap>(
            this,
            "navigate_to_map",
            std::bind(&MultiMapActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiMapActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiMapActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Initialize Nav2 client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Initialize initial pose publisher
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

        // Timer to publish initial pose once after 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                set_initial_pose(-6.0, 0.0, 0.0); // x, y, yaw in radians (edit as needed)
                timer_->cancel(); // Only run once
            }
        );

        RCLCPP_INFO(this->get_logger(), "Nav Server initialized (Nav2 forwarding + initial pose on startup)");
    }

    ~MultiMapActionServer() {
        RCLCPP_INFO(this->get_logger(), "Shutting down Nav Action Server");
        nav2_client_.reset();
        action_server_.reset();
    }

private:
    // ROS 2 Components
    rclcpp_action::Server<NavigateToMap>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToMap::Goal>)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToMap>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
    {
        // Use a thread to avoid blocking the executor
        std::thread{
            [this, goal_handle]() {
                execute(goal_handle);
            }
        }.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToMap::Feedback>();
        auto result = std::make_shared<NavigateToMap::Result>();

        try {
            feedback->phase = "Forwarding to Nav2";
            goal_handle->publish_feedback(feedback);

            if (send_goal_to_nav2(goal->target_pose, goal_handle)) {
                result->success = true;
                goal_handle->succeed(result);
            } else {
                result->success = false;
                goal_handle->abort(result);
            }
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Exception in execution: %s", ex.what());
            result->success = false;
            goal_handle->abort(result);
        }
    }

    bool send_goal_to_nav2(
        const geometry_msgs::msg::PoseStamped &pose,
        const std::shared_ptr<GoalHandleNavigateToMap> &goal_handle)
    {
        if (!nav2_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return false;
        }

        auto nav_goal = NavigateToPose::Goal();
        nav_goal.pose = pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = [this, goal_handle](
            rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            auto multi_map_feedback = std::make_shared<NavigateToMap::Feedback>();
            multi_map_feedback->phase = "Navigating: " + std::to_string(feedback->distance_remaining) + "m remaining";
            goal_handle->publish_feedback(multi_map_feedback);
        };

        auto future_goal = nav2_client_->async_send_goal(nav_goal, send_goal_options);

        // Wait for goal acceptance
        auto start_time = this->now();
        while (rclcpp::ok() &&
               future_goal.wait_for(100ms) != std::future_status::ready &&
               (this->now() - start_time) < 5s) {
            // Non-blocking wait
        }

        if (!future_goal.valid()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2");
            return false;
        }

        auto goal_handle_nav2 = future_goal.get();
        if (!goal_handle_nav2) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2");
            return false;
        }

        // Wait for result
        auto result_future = nav2_client_->async_get_result(goal_handle_nav2);
        start_time = this->now();
        while (rclcpp::ok() &&
               result_future.wait_for(100ms) != std::future_status::ready &&
               (this->now() - start_time) < 30s) {
            // Non-blocking wait
        }

        if (!result_future.valid()) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action timed out");
            return false;
        }

        auto wrapped_result = result_future.get();
        return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    void set_initial_pose(double x, double y, double yaw_rad) {
        geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
        init_pose.header.stamp = this->now();
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.position.x = x;
        init_pose.pose.pose.position.y = y;
        init_pose.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        init_pose.pose.pose.orientation.x = q.x();
        init_pose.pose.pose.orientation.y = q.y();
        init_pose.pose.pose.orientation.z = q.z();
        init_pose.pose.pose.orientation.w = q.w();

        // Recommended covariance (AMCL expects these)
        for (int i = 0; i < 36; i++) init_pose.pose.covariance[i] = 0.0;
        init_pose.pose.covariance[0] = 0.25;    // X
        init_pose.pose.covariance[7] = 0.25;    // Y
        init_pose.pose.covariance[35] = 0.0685; // Yaw

        initialpose_pub_->publish(init_pose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%.2f y=%.2f yaw=%.2f deg", x, y, yaw_rad * 180.0 / M_PI);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

