#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_server/action/navigate_to_map.hpp"
#include "multi_map_manager/action/multi_map_navigate.hpp"
#include "multi_map_manager/srv/get_wormhole.hpp"

using namespace std::chrono_literals;
using MultiMapNavigate = multi_map_manager::action::MultiMapNavigate;
using NavigateToMap = nav_server::action::NavigateToMap;
using GoalHandleMultiMapNavigate = rclcpp_action::ServerGoalHandle<MultiMapNavigate>;
using GetWormhole = multi_map_manager::srv::GetWormhole;

class MultiMapManager : public rclcpp::Node {
public:
    MultiMapManager()
    : Node("multi_map_manager")
    {
        action_server_ = rclcpp_action::create_server<MultiMapNavigate>(
            this, "multi_map_navigate",
            std::bind(&MultiMapManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiMapManager::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiMapManager::handle_accepted, this, std::placeholders::_1)
        );

        nav_server_client_ = rclcpp_action::create_client<NavigateToMap>(this, "navigate_to_map");
        wormhole_client_ = this->create_client<GetWormhole>("get_wormhole");
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

        // === Update these with your actual map file paths ===
        map_yaml_["room1"] = "/home/ubuntu-desk/multi_nav/src/maps/room1_map.yaml";
        map_yaml_["room2"] = "/home/ubuntu-desk/multi_nav/src/maps/room2_map.yaml";
        map_yaml_["room3"] = "/home/ubuntu-desk/multi_nav/src/maps/room3_map.yaml";
        // ================================================

        current_map_ = "room1";  // Start map
    }

private:
    rclcpp_action::Server<MultiMapNavigate>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToMap>::SharedPtr nav_server_client_;
    rclcpp::Client<GetWormhole>::SharedPtr wormhole_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    std::map<std::string, std::string> map_yaml_;
    std::string current_map_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiMapNavigate::Goal> goal) {
        if (!goal) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMultiMapNavigate>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ========== Main Orchestration ==========
    void handle_accepted(const std::shared_ptr<GoalHandleMultiMapNavigate> goal_handle) {
        std::thread([this, goal_handle]() {
            const auto goal = goal_handle->get_goal();
            const std::string target_map = goal->target_map;
            const geometry_msgs::msg::PoseStamped target_pose = goal->target_pose;

            auto result = std::make_shared<MultiMapNavigate::Result>();

            if (target_map == current_map_) {
                // Navigation within current map
                goal_handle->publish_feedback(feedback_msg("Navigating in current map..."));
                bool nav_ok = send_nav_goal(current_map_, target_pose, goal_handle);
                if (nav_ok) {
                    result->success = true;
                    result->message = "Arrived at goal in current map.";
                    goal_handle->succeed(result);
                } else {
                    result->success = false;
                    result->message = "Failed to reach goal in current map.";
                    goal_handle->abort(result);
                }
                return;
            }

            // Navigation between maps (via wormhole)
            geometry_msgs::msg::PoseStamped entry_pose;
            double exit_x, exit_y, exit_yaw;
            goal_handle->publish_feedback(feedback_msg("Requesting wormhole info..."));
            if (!get_wormhole(current_map_, target_map, entry_pose, exit_x, exit_y, exit_yaw)) {
                result->success = false;
                result->message = "Failed to get wormhole info";
                goal_handle->abort(result);
                return;
            }

            // 1. Go to wormhole entry
            goal_handle->publish_feedback(feedback_msg("Navigating to wormhole entry in " + current_map_));
            if (!send_nav_goal(current_map_, entry_pose, goal_handle)) {
                result->success = false;
                result->message = "Failed to reach wormhole entry in " + current_map_;
                goal_handle->abort(result);
                return;
            }

            // 2. Load target map
            goal_handle->publish_feedback(feedback_msg("Switching to map: " + target_map));
            if (!load_map(map_yaml_.at(target_map))) {
                result->success = false;
                result->message = "Failed to load map: " + target_map;
                goal_handle->abort(result);
                return;
            }
            current_map_ = target_map; // Update state

            // 3. Set initial pose at wormhole exit
            goal_handle->publish_feedback(feedback_msg("Setting initial pose at wormhole exit"));
            set_initial_pose(exit_x, exit_y, exit_yaw);
            std::this_thread::sleep_for(2s); // Give time for localization to settle

            // 4. Navigate to final goal in target map
            goal_handle->publish_feedback(feedback_msg("Navigating to final goal in " + target_map));
            if (!send_nav_goal(target_map, target_pose, goal_handle)) {
                result->success = false;
                result->message = "Failed to reach final goal in " + target_map;
                goal_handle->abort(result);
                return;
            }

            result->success = true;
            result->message = "Arrived at goal in " + target_map;
            goal_handle->succeed(result);

        }).detach();
    }

    MultiMapNavigate::Feedback::SharedPtr feedback_msg(const std::string &phase) {
        auto feedback = std::make_shared<MultiMapNavigate::Feedback>();
        feedback->phase = phase;
        return feedback;
    }

    // ========== Service/Action Utilities ==========
    bool get_wormhole(const std::string& from_room, const std::string& to_room,
                      geometry_msgs::msg::PoseStamped &entry_pose,
                      double &exit_x, double &exit_y, double &exit_yaw)
    {
        auto request = std::make_shared<GetWormhole::Request>();
        request->from_room = from_room;
        request->to_room = to_room;

        if (!wormhole_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Wormhole service not available");
            return false;
        }
        auto future = wormhole_client_->async_send_request(request);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get wormhole info");
            return false;
        }
        auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Wormhole response indicated failure.");
            return false;
        }
        entry_pose = response->entry_pose;
        exit_x = response->exit_x;
        exit_y = response->exit_y;
        exit_yaw = response->exit_yaw;
        return true;
    }

    bool send_nav_goal(const std::string& map_name, const geometry_msgs::msg::PoseStamped &goal_pose,
                       const std::shared_ptr<GoalHandleMultiMapNavigate>& goal_handle)
    {
        if (!nav_server_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "nav_server action server not available!");
            return false;
        }
        auto nav_goal = NavigateToMap::Goal();
        nav_goal.target_map = map_name;
        nav_goal.target_pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToMap>::SendGoalOptions();
        std::promise<bool> finished;
        send_goal_options.result_callback = [&finished](const auto &) { finished.set_value(true); };
        auto future_goal_handle = nav_server_client_->async_send_goal(nav_goal, send_goal_options);
        if (future_goal_handle.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to nav_server.");
            return false;
        }
        auto nav_goal_handle = future_goal_handle.get();
        if (!nav_goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by nav_server.");
            return false;
        }
        // Wait for the result
        auto result_future = nav_server_client_->async_get_result(nav_goal_handle);
        if (result_future.wait_for(120s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for nav_server result.");
            return false;
        }
        auto wrapped_result = result_future.get();
        if (wrapped_result.result->success) {
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "nav_server navigation failed: %s", wrapped_result.result->message.c_str());
            return false;
        }
    }

    bool load_map(const std::string& map_yaml_path)
    {
        auto client = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
        if (!client->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "map_server/load_map service not available!");
            return false;
        }
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_yaml_path;
        auto result_future = client->async_send_request(request);
        if (result_future.wait_for(10s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for map to load.");
            return false;
        }
        auto response = result_future.get();
        if (response->result != 0) {  // 0 is RESULT_SUCCESS
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", map_yaml_path.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map: %s", map_yaml_path.c_str());
        return true;
    }

    void set_initial_pose(double x, double y, double yaw = 0.0)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.z = std::sin(yaw/2.0);
        msg.pose.pose.orientation.w = std::cos(yaw/2.0);
        for (int i = 0; i < 36; i++) msg.pose.covariance[i] = 0.0;
        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.0685;
        initialpose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Initial pose set at (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

