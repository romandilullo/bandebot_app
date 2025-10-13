/*****************************************************************************
 * 
 * Copyright 2025 Roman Di Lullo
 * 
 * Created: 01/10/2025
 * Last Modified: 13/10/2025
 * 
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <rogue_droids_interfaces/msg/can_frame.hpp>
#include <rogue_droids_interfaces/srv/find_free_spot.hpp>
#include <rogue_droids_interfaces/srv/get_current_pose.hpp>
#include <rogue_droids_interfaces/srv/start_catering.hpp>
#include <rogue_droids_interfaces/srv/stop_catering.hpp>
#include <rogue_droids_interfaces/srv/start_serving.hpp>
#include <rogue_droids_interfaces/srv/stop_serving.hpp>
#include <rogue_droids_interfaces/action/go_to_relative_pose.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "../../mulita/include/mulita_defines.h"
#include "../../mulita/include/bandebot_defines.h"

using namespace std::chrono_literals;

#define MIN_CATERING_TIME 10.0
#define MAX_CATERING_TIME 120.0
#define MIN_CATERING_RADIUS 0.5
#define MAX_CATERING_RADIUS 5.0

#define MAX_TIME_REACHING_POSE 45.0
#define MAX_TIME_WAITING_SERVING_START_STOP 3.0
#define MAX_TIME_WAITING_NAV_CANCEL 3.0
#define MAX_TIME_WAITING_SPOT_SEARCH 3.0

class CateringNode : public rclcpp::Node {
public:
    CateringNode() : Node("catering_node") {
        // Initialize state variables
        current_state_ = BANDEBOT_CATERING_STATE::Standby;
        current_app_state_ = BANDEBOT_APP_STATE::Unknown;
        
        // Create subscriber to bandebot app state
        app_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/app_state", 10, 
            std::bind(&CateringNode::app_state_callback, this, std::placeholders::_1));
        
        // Create publisher for catering mode state
        catering_mode_state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(
            "bandebot/catering_mode_state", 10);
        
        // Create service servers
        start_catering_service_ = this->create_service<rogue_droids_interfaces::srv::StartCatering>(
            std::string("bandebot/start_catering"),
            std::bind(&CateringNode::handle_start_catering, this, std::placeholders::_1, std::placeholders::_2));
        
        stop_catering_service_ = this->create_service<rogue_droids_interfaces::srv::StopCatering>(
            std::string("bandebot/stop_catering"), 
            std::bind(&CateringNode::handle_stop_catering, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create service clients for bandebot serving
        start_serving_client_ = this->create_client<rogue_droids_interfaces::srv::StartServing>("bandebot/start_serving");
        stop_serving_client_ = this->create_client<rogue_droids_interfaces::srv::StopServing>("bandebot/stop_serving");
        
        // Create service client for finding free spots
        find_free_spot_client_ = this->create_client<rogue_droids_interfaces::srv::FindFreeSpot>("mulita/find_free_spot");
        
        // Create service client for getting current pose
        get_current_pose_client_ = this->create_client<rogue_droids_interfaces::srv::GetCurrentPose>("mulita/get_current_pose");
        
        // Create action client for navigation
        navigation_action_client_ = rclcpp_action::create_client<rogue_droids_interfaces::action::GoToRelativePose>(
            this, "go_to_relative_pose");
        
        // Create timer for state machine
        state_timer_ = this->create_wall_timer(
            100ms, std::bind(&CateringNode::state_machine_callback, this));
        
        // Publish initial catering state
        setState(BANDEBOT_CATERING_STATE::Standby);
        
        RCLCPP_INFO(this->get_logger(), "Catering node initialized - State: Standby");
    }

private:
    // State variables
    BANDEBOT_CATERING_STATE current_state_;
    BANDEBOT_APP_STATE current_app_state_;
    
    // Catering parameters
    float catering_radius_;
    float on_spot_catering_time_;
    std::chrono::steady_clock::time_point state_start_time_;
    
    // Current pose storage
    geometry_msgs::msg::Pose current_pose_;
    bool pose_valid_ = false;
    
    // ROS2 components
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr app_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr catering_mode_state_publisher_;
    rclcpp::Service<rogue_droids_interfaces::srv::StartCatering>::SharedPtr start_catering_service_;
    rclcpp::Service<rogue_droids_interfaces::srv::StopCatering>::SharedPtr stop_catering_service_;
    rclcpp::Client<rogue_droids_interfaces::srv::StartServing>::SharedPtr start_serving_client_;
    rclcpp::Client<rogue_droids_interfaces::srv::StopServing>::SharedPtr stop_serving_client_;
    rclcpp::Client<rogue_droids_interfaces::srv::FindFreeSpot>::SharedPtr find_free_spot_client_;
    rclcpp::Client<rogue_droids_interfaces::srv::GetCurrentPose>::SharedPtr get_current_pose_client_;
    rclcpp_action::Client<rogue_droids_interfaces::action::GoToRelativePose>::SharedPtr navigation_action_client_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    // Action state variables
    rclcpp_action::ClientGoalHandle<rogue_droids_interfaces::action::GoToRelativePose>::SharedPtr current_goal_handle_;
    bool action_completed_ = false;
    bool action_succeeded_ = false;
    
    void app_state_callback(const rogue_droids_interfaces::msg::CanFrame::SharedPtr msg) {
        if (msg->len >= 1) {
            current_app_state_ = static_cast<BANDEBOT_APP_STATE>(msg->data[0]);
            RCLCPP_DEBUG(this->get_logger(), "App state updated: %u", 
                        static_cast<uint8_t>(current_app_state_));
        }
    }
    
    void handle_start_catering(
        const std::shared_ptr<rogue_droids_interfaces::srv::StartCatering::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StartCatering::Response> response) {
        
        // Check preconditions
        if (current_state_ != BANDEBOT_CATERING_STATE::Standby) {
            response->success = false;
            response->message = "Catering node is not in Standby state";
            RCLCPP_WARN(this->get_logger(), "Start catering rejected: Not in Standby state");
            return;
        }
        
        if (current_app_state_ != BANDEBOT_APP_STATE::ReadyLoaded) {
            response->success = false;
            response->message = "App state is not READY_LOADED";
            RCLCPP_WARN(this->get_logger(), "Start catering rejected: App state is not READY_LOADED");
            return;
        }
        
        // Validate parameters
        if (request->catering_radius < MIN_CATERING_RADIUS || request->catering_radius > MAX_CATERING_RADIUS ||
            request->on_spot_catering_time < MIN_CATERING_TIME || request->on_spot_catering_time > MAX_CATERING_TIME) {
            response->success = false;
            response->message = "Invalid catering parameters (radius between 0.5-5.0m, time between 5-60s)";
            RCLCPP_WARN(this->get_logger(), "Start catering rejected: Invalid parameters");
            return;
        }
        
        // Store catering parameters
        catering_radius_ = request->catering_radius;
        on_spot_catering_time_ = request->on_spot_catering_time;

        // Get current pose and store locally
        call_get_current_pose();

        response->success = true;
        response->message = "Catering started successfully";
        RCLCPP_INFO(this->get_logger(), "Catering started - Radius: %.2fm, On-spot time: %.2fs", 
                   catering_radius_, on_spot_catering_time_);
    }
    
    void handle_stop_catering(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopCatering::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopCatering::Response> response) {
        
        (void)request; // Suppress unused parameter warning
        
        if (current_state_ == BANDEBOT_CATERING_STATE::Standby) {
            response->success = false;
            response->message = "Catering is not active";
            RCLCPP_WARN(this->get_logger(), "Stop catering rejected: Already in Standby");
            return;
        }
        
        // Can only stop catering if in ServingFood state
        if (current_state_ != BANDEBOT_CATERING_STATE::ServingFood) {
            response->success = false;
            response->message = "Can only stop catering when in ServingFood state";
            RCLCPP_WARN(this->get_logger(), "Stop catering rejected: Not in ServingFood state");
            return;
        }
        
        // Call stop serving before returning to Standby
        call_stop_serving();
        
        // Return to Standby state
        setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
        
        response->success = true;
        response->message = "Catering stopped successfully";
        RCLCPP_INFO(this->get_logger(), "Catering stopped");
    }
    
    void call_start_serving() {
        if (!start_serving_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "bandebot/start_serving service not available");
            return;
        }
        
        auto request = std::make_shared<rogue_droids_interfaces::srv::StartServing::Request>();
        request->mode = 0; // Default mode
        
        start_serving_client_->async_send_request(request,
            [this](rclcpp::Client<rogue_droids_interfaces::srv::StartServing>::SharedFuture result) {
            try {
                auto response = result.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully called bandebot/start_serving");
                } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to call bandebot/start_serving");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception calling bandebot/start_serving: %s", e.what());
            }
        });
    }
    
    void call_stop_serving() {
        if (!stop_serving_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "bandebot/stop_serving service not available");
            return;
        }
        
        auto request = std::make_shared<rogue_droids_interfaces::srv::StopServing::Request>();
        request->mode = 0; // Default mode
        
        stop_serving_client_->async_send_request(request,
            [this](rclcpp::Client<rogue_droids_interfaces::srv::StopServing>::SharedFuture result) {
                try {
                    auto response = result.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully called bandebot/stop_serving");
            } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to call bandebot/stop_serving");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception calling bandebot/stop_serving: %s", e.what());
        }
            });
    }
    
    void call_get_current_pose() {
        if (!get_current_pose_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "mulita/get_current_pose service not available");
            pose_valid_ = false;
            return;
        }
        
        setState(BANDEBOT_CATERING_STATE::FindingCurrentPose);

        auto request = std::make_shared<rogue_droids_interfaces::srv::GetCurrentPose::Request>();
        
        get_current_pose_client_->async_send_request(request,
            [this](rclcpp::Client<rogue_droids_interfaces::srv::GetCurrentPose>::SharedFuture result) {
                try {
                    auto response = result.get();
                    if (response->success) {
                        current_pose_ = response->pose;
                        pose_valid_ = true;
                        RCLCPP_INFO(this->get_logger(), "Current pose updated: x=%.2f, y=%.2f, z=%.2f", 
                                   current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
                        RCLCPP_DEBUG(this->get_logger(), "Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                                    current_pose_.orientation.x, current_pose_.orientation.y,
                                    current_pose_.orientation.z, current_pose_.orientation.w);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to get current pose: %s", response->message.c_str());
                        pose_valid_ = false;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception calling get_current_pose: %s", e.what());
                    pose_valid_ = false;
                }
            });
    }
    
    void call_find_free_spot() {
        if (!find_free_spot_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "mulita/find_free_spot service not available");
            setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
            return;
        }

        setState(BANDEBOT_CATERING_STATE::FindingEmptySpot);
        
        auto request = std::make_shared<rogue_droids_interfaces::srv::FindFreeSpot::Request>();
        request->radius = catering_radius_; // Use the catering radius parameter
        
        find_free_spot_client_->async_send_request(request,
            [this](rclcpp::Client<rogue_droids_interfaces::srv::FindFreeSpot>::SharedFuture result) {
                try {
                    auto response = result.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Free spot found at x=%.2f, y=%.2f, section=%d", 
                                   response->relative_x, response->relative_y, response->section_id);
                        // Call navigation with returned values
                        call_navigation_action(response->relative_x, response->relative_y, M_PI);
                        setState(BANDEBOT_CATERING_STATE::MovingToEmptySpot);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to find free spot: %s. Using fallback navigation.", 
                                   response->message.c_str());
                        // Fallback navigation if no free spot found
                        call_navigation_action(0.0, 0.0, M_PI);
                        setState(BANDEBOT_CATERING_STATE::Turning180Degrees);
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception calling find_free_spot: %s", e.what());
                    // Fallback navigation on exception
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                }
            });
    }
    
    void call_navigation_action(double x, double y, double theta) {
        if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Navigation action server not available");
            action_completed_ = true;
            action_succeeded_ = false;
            return;
        }
        
        auto goal_msg = rogue_droids_interfaces::action::GoToRelativePose::Goal();
        goal_msg.x = x;
        goal_msg.y = y;
        goal_msg.theta = theta;
        
        auto send_goal_options = rclcpp_action::Client<rogue_droids_interfaces::action::GoToRelativePose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<rogue_droids_interfaces::action::GoToRelativePose>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                    action_completed_ = true;
                    action_succeeded_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                    current_goal_handle_ = goal_handle;
                }
            };
            
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<rogue_droids_interfaces::action::GoToRelativePose>::SharedPtr,
                   const std::shared_ptr<const rogue_droids_interfaces::action::GoToRelativePose::Feedback> feedback) {
                RCLCPP_DEBUG(this->get_logger(), "Navigation feedback: distance remaining: %.2f", 
                           feedback->distance_remaining);
            };
            
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<rogue_droids_interfaces::action::GoToRelativePose>::WrappedResult & result) {
                action_completed_ = true;
                switch(result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Navigation action succeeded");
                        action_succeeded_ = true;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), "Navigation action was aborted");
                        action_succeeded_ = false;
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Navigation action was canceled");
                        action_succeeded_ = false;
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown navigation action result code");
                        action_succeeded_ = false;
                        break;
                }
                current_goal_handle_.reset();
            };
        
        // Reset action state
        action_completed_ = false;
        action_succeeded_ = false;
        
        // Send the goal (async_send_goal returns a future, we handle it in the goal_response_callback)
        navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
        
        RCLCPP_INFO(this->get_logger(), "Sent navigation goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    }
    
    void cancel_navigation_action() {
        if (current_goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "Canceling navigation action");
            
            // Use async_cancel_goal with a callback to handle the cancellation result
            navigation_action_client_->async_cancel_goal(current_goal_handle_,
                [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel_response) {
                    if (cancel_response) {
                        switch (cancel_response->return_code) {
                            case action_msgs::srv::CancelGoal_Response::ERROR_NONE:
                                RCLCPP_INFO(this->get_logger(), "Goal cancellation accepted");
                                action_completed_ = true;
                                action_succeeded_ = false;
                                break;
                            case action_msgs::srv::CancelGoal_Response::ERROR_REJECTED:
                                RCLCPP_WARN(this->get_logger(), "Goal cancellation rejected");
                                // Force completion since cancellation was rejected
                                action_completed_ = true;
                                action_succeeded_ = false;
                                current_goal_handle_.reset();
                                break;
                            case action_msgs::srv::CancelGoal_Response::ERROR_UNKNOWN_GOAL_ID:
                                RCLCPP_WARN(this->get_logger(), "Goal cancellation failed: Unknown goal ID");
                                // Force completion since goal is unknown
                                action_completed_ = true;
                                action_succeeded_ = false;
                                current_goal_handle_.reset();
                                break;
                            default:
                                RCLCPP_ERROR(this->get_logger(), "Unknown cancellation response code: %d", cancel_response->return_code);
                                // Force completion on unknown response
                                action_completed_ = true;
                                action_succeeded_ = false;
                                current_goal_handle_.reset();
                                break;
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Null cancel response received");
                        // Force completion on null response
                        action_completed_ = true;
                        action_succeeded_ = false;
                        current_goal_handle_.reset();
                    }
                });
        } else {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
            // Ensure action is marked as completed if there's no goal handle
            action_completed_ = true;
            action_succeeded_ = false;
        }
    }
    
    void setState(BANDEBOT_CATERING_STATE new_state) {
        if (current_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", 
                       getStateName(current_state_).c_str(), 
                       getStateName(new_state).c_str());
            current_state_ = new_state;
            state_start_time_ = std::chrono::steady_clock::now();
            
            // Publish the new catering state
            publishCateringState();
        }
    }
    
    void publishCateringState() {
        auto ros_msg = std_msgs::msg::UInt8();
        ros_msg.data = static_cast<uint8_t>(current_state_);
        catering_mode_state_publisher_->publish(ros_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published catering state: %u (%s)", 
                    static_cast<uint8_t>(current_state_), getStateName(current_state_).c_str());
    }
    
    std::string getStateName(BANDEBOT_CATERING_STATE state) {
        switch(state) {
            case BANDEBOT_CATERING_STATE::Standby: return "Standby";
            case BANDEBOT_CATERING_STATE::FindingEmptySpot: return "FindingEmptySpot";
            case BANDEBOT_CATERING_STATE::FindingCurrentPose: return "FindingCurrentPose";
            case BANDEBOT_CATERING_STATE::MovingToEmptySpot: return "MovingToEmptySpot";
            case BANDEBOT_CATERING_STATE::WaitingNavigationCancel: return "WaitingNavigationCancel";
            case BANDEBOT_CATERING_STATE::WaitingServingStarted: return "WaitingServingStarted";
            case BANDEBOT_CATERING_STATE::ServingFood: return "ServingFood";
            case BANDEBOT_CATERING_STATE::WaitingServingStopped: return "WaitingServingStopped";
            case BANDEBOT_CATERING_STATE::ReturningToStandby: return "ReturningToStandby";
            case BANDEBOT_CATERING_STATE::Turning180Degrees: return "Turning180Degrees";
            default: return "Unknown";
        }
    }

    void state_machine_callback() {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_in_state = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - state_start_time_).count();
        
        switch (current_state_) {
            case BANDEBOT_CATERING_STATE::Standby:
                // No action needed in standby
                break;

            case BANDEBOT_CATERING_STATE::FindingCurrentPose:

                if(pose_valid_) {
                    call_find_free_spot();
                    RCLCPP_INFO(this->get_logger(), "Current pose valid, proceeding to find empty spot");
                    break;
                }   

                if (elapsed_in_state >= MAX_TIME_WAITING_SPOT_SEARCH) { 
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                    RCLCPP_INFO(this->get_logger(), "Timout searching for POSE..");
                }
                break;
                
            case BANDEBOT_CATERING_STATE::FindingEmptySpot:

                if (elapsed_in_state >= MAX_TIME_WAITING_SPOT_SEARCH) {
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                    RCLCPP_INFO(this->get_logger(), "Timout searching for empty spot...");
                }
                break;

            case BANDEBOT_CATERING_STATE::Turning180Degrees:

                if (action_completed_) {

                    if (action_succeeded_) {
                        RCLCPP_INFO(this->get_logger(), "Successfully arrived at target position");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Navigation action failed");
                    }

                    call_find_free_spot();
                    RCLCPP_INFO(this->get_logger(), "Finished Turning 180 degrees, now finding empty spot");

                } else if (elapsed_in_state >= MAX_TIME_REACHING_POSE) {

                    RCLCPP_WARN(this->get_logger(), "Timeout reaching POSE, canceling action");
                    cancel_navigation_action();
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                }
                break;

            case BANDEBOT_CATERING_STATE::MovingToEmptySpot:

                // Monitor navigation action and timeout
                if (action_completed_) {

                    if (action_succeeded_) {
                        RCLCPP_INFO(this->get_logger(), "Successfully arrived at target position");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Navigation action failed");
                    }

                    // Either way, proceed to serving food
                    setState(BANDEBOT_CATERING_STATE::WaitingServingStarted);
                    call_start_serving();
                    RCLCPP_INFO(this->get_logger(), "POSE reached, now serving food");

                } else if (elapsed_in_state >= MAX_TIME_REACHING_POSE) {

                    RCLCPP_WARN(this->get_logger(), "Timeout reaching POSE, canceling action");
                    cancel_navigation_action();
                    setState(BANDEBOT_CATERING_STATE::WaitingNavigationCancel);
                }
                break;

            case BANDEBOT_CATERING_STATE::WaitingNavigationCancel:

                // Waiting for navigation cancel to complete
                if (action_completed_) {

                    call_start_serving();
                    setState(BANDEBOT_CATERING_STATE::WaitingServingStarted);
                    RCLCPP_INFO(this->get_logger(), "Now serving food after canceling navigation");

                } else if (elapsed_in_state >= MAX_TIME_WAITING_NAV_CANCEL) {
                    RCLCPP_WARN(this->get_logger(), "Navigation cancel timeout reached, forcing state change");
                    cancel_navigation_action();
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                }
                break;

            case BANDEBOT_CATERING_STATE::WaitingServingStarted:

                if( current_app_state_ == BANDEBOT_APP_STATE::Serving) {

                    setState(BANDEBOT_CATERING_STATE::ServingFood);

                } else if (elapsed_in_state >= MAX_TIME_WAITING_SERVING_START_STOP) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for serving to start");
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                }   
                break;

            case BANDEBOT_CATERING_STATE::ServingFood:

                // Check if on-spot catering time has elapsed
                if (elapsed_in_state >= on_spot_catering_time_) {
                    call_stop_serving();
                    call_find_free_spot();
                    RCLCPP_INFO(this->get_logger(), "On-spot catering time completed, finding new spot");
                }
                
                // Safety check: if app state changes away from Serving during ServingFood
                if ((elapsed_in_state >= MIN_CATERING_TIME) && (current_app_state_ != BANDEBOT_APP_STATE::Serving)) {
                    call_stop_serving();
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                    RCLCPP_WARN(this->get_logger(), "Stop: App state changed from Serving while serving food");
                    return; // Exit early to avoid the general safety check below
                }
                break;

            case BANDEBOT_CATERING_STATE::WaitingServingStopped:

                if( current_app_state_ == BANDEBOT_APP_STATE::ReadyLoaded) {

                    call_find_free_spot();

                } else if (elapsed_in_state >= MAX_TIME_WAITING_SERVING_START_STOP) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for serving to stop");
                    setState(BANDEBOT_CATERING_STATE::ReturningToStandby);
                }   
                break;


            case BANDEBOT_CATERING_STATE::ReturningToStandby:

                // TODO: Implement logic to return to standby position if needed
                // For now, we just transition to Standby after a brief wait
                if (elapsed_in_state >= 2) { // Wait 2 seconds before going to
                    setState(BANDEBOT_CATERING_STATE::Standby);
                }
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CateringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}