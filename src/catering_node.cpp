/*****************************************************************************
 * 
 * Copyright 2025 Roman Di Lullo
 * 
 * Created: 01/10/2025
 * Last Modified: 01/10/2025
 * 
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rogue_droids_interfaces/msg/can_frame.hpp>
#include <rogue_droids_interfaces/srv/start_catering.hpp>
#include <rogue_droids_interfaces/srv/stop_catering.hpp>
#include "../../mulita/include/mulita_defines.h"
#include "../../mulita/include/bandebot_defines.h"

using namespace std::chrono_literals;

enum class CateringState : uint8_t {
    Standby = 0,
    FindingEmptySpot = 1,
    MovingToEmptySpot = 2,
    CallingAttention = 3,
    ServingFood = 4
};

class CateringNode : public rclcpp::Node {
public:
    CateringNode() : Node("catering_node") {
        // Initialize state variables
        current_state_ = CateringState::Standby;
        current_app_state_ = BANDEBOT_APP_STATE::Unknown;
        
        // Create subscriber to bandebot app state
        app_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/app_state", 10, 
            std::bind(&CateringNode::app_state_callback, this, std::placeholders::_1));
        
        // Create service servers
        start_catering_service_ = this->create_service<rogue_droids_interfaces::srv::StartCatering>(
            "start_catering",
            std::bind(&CateringNode::handle_start_catering, this, std::placeholders::_1, std::placeholders::_2));
        
        stop_catering_service_ = this->create_service<rogue_droids_interfaces::srv::StopCatering>(
            "stop_catering", 
            std::bind(&CateringNode::handle_stop_catering, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create timer for state machine
        state_timer_ = this->create_wall_timer(
            100ms, std::bind(&CateringNode::state_machine_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Catering node initialized - State: Standby");
    }

private:
    // State variables
    CateringState current_state_;
    BANDEBOT_APP_STATE current_app_state_;
    
    // Catering parameters
    float catering_radius_;
    float catering_time_;
    std::chrono::steady_clock::time_point catering_start_time_;
    std::chrono::steady_clock::time_point state_start_time_;
    
    // ROS2 components
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr app_state_subscriber_;
    rclcpp::Service<rogue_droids_interfaces::srv::StartCatering>::SharedPtr start_catering_service_;
    rclcpp::Service<rogue_droids_interfaces::srv::StopCatering>::SharedPtr stop_catering_service_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    
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
        if (current_state_ != CateringState::Standby) {
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
        if (request->catering_radius <= 0.0 || request->catering_time <= 0.0) {
            response->success = false;
            response->message = "Invalid catering parameters (must be > 0)";
            RCLCPP_WARN(this->get_logger(), "Start catering rejected: Invalid parameters");
            return;
        }
        
        // Store catering parameters
        catering_radius_ = request->catering_radius;
        catering_time_ = request->catering_time;
        catering_start_time_ = std::chrono::steady_clock::now();
        
        // Transition to FindingEmptySpot state
        setState(CateringState::FindingEmptySpot);
        
        response->success = true;
        response->message = "Catering started successfully";
        RCLCPP_INFO(this->get_logger(), "Catering started - Radius: %.2fm, Time: %.2fs", 
                   catering_radius_, catering_time_);
    }
    
    void handle_stop_catering(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopCatering::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopCatering::Response> response) {
        
        (void)request; // Suppress unused parameter warning
        
        if (current_state_ == CateringState::Standby) {
            response->success = false;
            response->message = "Catering is not active";
            RCLCPP_WARN(this->get_logger(), "Stop catering rejected: Already in Standby");
            return;
        }
        
        // Return to Standby state
        setState(CateringState::Standby);
        
        response->success = true;
        response->message = "Catering stopped successfully";
        RCLCPP_INFO(this->get_logger(), "Catering stopped");
    }
    
    void setState(CateringState new_state) {
        if (current_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", 
                       getStateName(current_state_).c_str(), 
                       getStateName(new_state).c_str());
            current_state_ = new_state;
            state_start_time_ = std::chrono::steady_clock::now();
        }
    }
    
    std::string getStateName(CateringState state) {
        switch(state) {
            case CateringState::Standby: return "Standby";
            case CateringState::FindingEmptySpot: return "FindingEmptySpot";
            case CateringState::MovingToEmptySpot: return "MovingToEmptySpot";
            case CateringState::CallingAttention: return "CallingAttention";
            case CateringState::ServingFood: return "ServingFood";
            default: return "Unknown";
        }
    }
    
    void state_machine_callback() {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_since_start = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - catering_start_time_).count();
        auto elapsed_in_state = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - state_start_time_).count();
        
        switch (current_state_) {
            case CateringState::Standby:
                // No action needed in standby
                break;
                
            case CateringState::FindingEmptySpot:
                // Simulate finding an empty spot (placeholder logic)
                if (elapsed_in_state >= 3) { // 3 seconds to find a spot
                    setState(CateringState::MovingToEmptySpot);
                    RCLCPP_INFO(this->get_logger(), "Empty spot found, moving to position");
                }
                break;
                
            case CateringState::MovingToEmptySpot:
                // Simulate moving to the empty spot (placeholder logic)
                if (elapsed_in_state >= 5) { // 5 seconds to move
                    setState(CateringState::CallingAttention);
                    RCLCPP_INFO(this->get_logger(), "Arrived at position, calling attention");
                }
                break;
                
            case CateringState::CallingAttention:
                // Simulate calling attention (placeholder logic)
                if (elapsed_in_state >= 2) { // 2 seconds to call attention
                    setState(CateringState::ServingFood);
                    RCLCPP_INFO(this->get_logger(), "Attention called, now serving food");
                }
                break;
                
            case CateringState::ServingFood:
                // Check if catering time has elapsed
                if (elapsed_since_start >= catering_time_) {
                    setState(CateringState::Standby);
                    RCLCPP_INFO(this->get_logger(), "Catering time completed, returning to Standby");
                }
                break;
        }
        
        // Safety check: if app state changes away from ReadyLoaded during operation
        if (current_state_ != CateringState::Standby && 
            current_app_state_ != BANDEBOT_APP_STATE::ReadyLoaded) {
            setState(CateringState::Standby);
            RCLCPP_WARN(this->get_logger(), "Emergency stop: App state changed from ReadyLoaded");
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