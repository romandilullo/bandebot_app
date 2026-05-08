/*****************************************************************************
 * 
 * Copyright 2026 Roman Di Lullo
 * 
 * 
 * Created: 27/04/2026
 * Last Modified: 28/04/2026
 * 
 *****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "rogue_droids_interfaces/msg/can_frame.hpp"
#include "rogue_droids_interfaces/msg/robot_hardware_status.hpp"
#include "rogue_droids_interfaces/srv/set_sidelights.hpp"
#include "rogue_droids_interfaces/srv/start_serving.hpp"
#include "rogue_droids_interfaces/srv/stop_serving.hpp"

#include "../../mulita/include/mulita_defines.h"
#include "../../mulita/include/bandebot_defines.h"

#include "bandebot_app/bandebot_digital_twin.h"
#include "bandebot_app/colors_define.h"

class PromobotNode : public rclcpp::Node
{
public:
    PromobotNode() : Node("promobot")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Promobot Node...");

        // Initialize the node
        bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::WaitingMobileBaseReady;

        // Create publishers
        bandebot_app_state_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("bandebot/app_state", 10);
        robot_hardware_status_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::RobotHardwareStatus>("bandebot/robot_hardware_status", 10);
        power_relays_control_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("mulita/power_relays_control", 10);
        display_tray_lights_mode_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("bandebot/display_tray_lights_mode", 10);


        // Create subscribers

        mulita_robot_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/mobile_base_state", 10, std::bind(&PromobotNode::on_mulita_robot_state, this, std::placeholders::_1));
            
        power_supply_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/power_supply_state", 10, std::bind(&PromobotNode::on_power_supply_state, this, std::placeholders::_1));
    
        hw_id_information_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/hw_id_information", 10, std::bind(&PromobotNode::on_hw_id_information, this, std::placeholders::_1));

        // Create services
        srv_start_serving_ = this->create_service<rogue_droids_interfaces::srv::StartServing>(
            std::string("bandebot/start_serving"),
            std::bind(&PromobotNode::handle_srv_start_serving_, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_serving_ = this->create_service<rogue_droids_interfaces::srv::StopServing>(
            std::string("bandebot/stop_serving"),
            std::bind(&PromobotNode::handle_srv_stop_serving_, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create service client for sidelights control
        sidelights_client_ = this->create_client<rogue_droids_interfaces::srv::SetSidelights>("mulita/set_sidelights");
        
        operation_start_time_ = this->get_clock()->now();

        messageBandebotAppState();

        // Create a timer for the main loop function
        main_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&PromobotNode::mainLoopFunction, this));

        robot_state_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&PromobotNode::messageBandebotAppState, this));

        bandebot_hardware_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000),
            std::bind(&PromobotNode::messageBandebotHardwareStatus, this));

        RCLCPP_INFO(this->get_logger(), "Promobot node has been started.");
    }

private:

    BandebotTwin bandebot_twin_;
    BANDEBOT_APP_STATE last_bandebot_app_state_  = BANDEBOT_APP_STATE::Unknown;
    MULITA_ROBOT_STATE last_mobile_base_state_ = MULITA_ROBOT_STATE::Unknown;
    rclcpp::Time operation_start_time_;

    bool tray_content_state_updated = false;

    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr robot_state_update_timer_;
    rclcpp::TimerBase::SharedPtr bandebot_hardware_update_timer_;

    rclcpp::Service<rogue_droids_interfaces::srv::StartServing>::SharedPtr srv_start_serving_;
    rclcpp::Service<rogue_droids_interfaces::srv::StopServing>::SharedPtr srv_stop_serving_;
    
    // Service client for sidelights control
    rclcpp::Client<rogue_droids_interfaces::srv::SetSidelights>::SharedPtr sidelights_client_;

    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr bandebot_app_state_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::RobotHardwareStatus>::SharedPtr robot_hardware_status_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr power_relays_control_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr display_tray_lights_mode_publisher_;

    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr mulita_robot_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr power_supply_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr hw_id_information_subscriber_;


    void mainLoopFunction()
    {
        auto elapsed_time = this->get_clock()->now() - operation_start_time_;

        switch (bandebot_twin_.currentMobileBaseState)
        {
        case MULITA_ROBOT_STATE::Unknown:
            // No message has been received yet
            break;
 
        case MULITA_ROBOT_STATE::PowerOn:
        case MULITA_ROBOT_STATE::CheckingSystems:
        case MULITA_ROBOT_STATE::Calibrating:
            // Do nothing... wait for the mobile base to be ready
            break;
 
        case MULITA_ROBOT_STATE::ReadyStationary:
        case MULITA_ROBOT_STATE::ReadyMoving:

            // Normal operation
            switch (bandebot_twin_.currentBandebotState)
            {
            case BANDEBOT_APP_STATE::Unknown:
                break;

            case BANDEBOT_APP_STATE::WaitingMobileBaseReady:

                messagePowerRelayControl(true, true);

                bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::IdentifyingHardware;
                operation_start_time_ = this->get_clock()->now();                

                break;

            case BANDEBOT_APP_STATE::IdentifyingHardware:


                if( bandebot_twin_.IsPromobotHwReady() )
                {
                    bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ApplicationReady;
                    setSidelights(SIDELIGHTS_MODE::Off, SIDELIGHTS_COLOR::Off, 500);
                }
                else
                {
                    if( elapsed_time.seconds() > HW_IDENTIFICATION_TIMEOUT_SEC )
                    {
                        // TODO: Log error and go to safe state?
                        // RCLCPP_INFO(this->get_logger(), "Hardware identification timedout after %f seconds", elapsed_time.seconds());
                    }
                }
                break;

            case BANDEBOT_APP_STATE::Error:
                break;

            case BANDEBOT_APP_STATE::ApplicationReady:
                bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ReadyLoaded;
                break;


            case BANDEBOT_APP_STATE::ReadyLoaded:
                break;


            case BANDEBOT_APP_STATE::Serving:
                break;

            default:
                // Force unsupported legacy states into Error to keep the reduced state machine.
                bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::Error;
                break;
            }
    
            break;
 
        case MULITA_ROBOT_STATE::Error:
        default:

            // TODO: Handle unexpected mobile base states
            break;
        }

    }

    void on_mulita_robot_state(const rogue_droids_interfaces::msg::CanFrame msg) {

        if( msg.len >= 1 )
            bandebot_twin_.currentMobileBaseState = (MULITA_ROBOT_STATE)msg.data[0];
    }

    void on_power_supply_state(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {battery_voltage}, {current_ucu}, {current_ecus}, {current_motor}
        if( msg.len == 8)
            bandebot_twin_.ProcessPowerSupplyState(this->get_clock()->now(), uint16_t ((msg.data[1] << 8) | msg.data[0]),
                int16_t ((msg.data[3] << 8) | msg.data[2]), int16_t ((msg.data[5] << 8) | msg.data[4]),
                int16_t ((msg.data[7] << 8) | msg.data[6]));
    }

    void on_hw_id_information(const rogue_droids_interfaces::msg::CanFrame msg) {

        if( msg.len >= 4) {
            uint16_t error_count = static_cast<uint16_t>(msg.data[2]) | (static_cast<uint16_t>(msg.data[3]) << 8);
            bandebot_twin_.ProcessHwInformation(this->get_clock()->now(), msg.data[0], msg.data[1], error_count);
        }
    }

    void handle_srv_start_serving_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StartServing::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StartServing::Response> response)
    {
        response->success = bandebot_twin_.StartDisplayMode(request->mode);
    }
    
    void handle_srv_stop_serving_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopServing::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopServing::Response> response)
    {
        response->success = bandebot_twin_.StopDisplayMode(request->mode);
    }
    

    void messagePowerRelayControl(bool app_relay_state, bool aux_relay_state)
    {
        auto ros_msg = rogue_droids_interfaces::msg::CanFrame();

        ros_msg.len = 2;
        ros_msg.data[0] = static_cast<uint8_t>(app_relay_state);
        ros_msg.data[1] = static_cast<uint8_t>(aux_relay_state);

        power_relays_control_publisher_->publish(ros_msg);
    }

    void messageBandebotAppState()
    {
        auto ros_msg = rogue_droids_interfaces::msg::CanFrame();

        ros_msg.len = 1;
        ros_msg.data[0] = static_cast<uint8_t>(bandebot_twin_.currentBandebotState);

        bandebot_app_state_publisher_->publish(ros_msg);


        // Update trays colours and sidelights based on state changes
        if((bandebot_twin_.currentBandebotState != last_bandebot_app_state_) || (bandebot_twin_.currentMobileBaseState != last_mobile_base_state_))
        {
            // Handle sidelights based on state transitions
            if (bandebot_twin_.currentBandebotState == BANDEBOT_APP_STATE::Serving) {
                setSidelights(SIDELIGHTS_MODE::On, SIDELIGHTS_COLOR::Green, 500);
            } else if (last_bandebot_app_state_ == BANDEBOT_APP_STATE::Serving) {
                setSidelights(SIDELIGHTS_MODE::Off, SIDELIGHTS_COLOR::Off, 500);
            }

            // Handle display tray lights based on state transitions
            if (bandebot_twin_.currentBandebotState == BANDEBOT_APP_STATE::ReadyLoaded) {

                if( bandebot_twin_.currentMobileBaseState == MULITA_ROBOT_STATE::ReadyMoving ) {

                    setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayUpper, TRAYLIGHTS_MODE::On, GenericColor::LIGHT_BLUE, 500);
                    setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayLower, TRAYLIGHTS_MODE::RGBPalette, GenericColor::WHITE, 500);

                } else {

                    setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayUpper, TRAYLIGHTS_MODE::On, GenericColor::LIGHT_BLUE, 500);
                    setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayLower, TRAYLIGHTS_MODE::On, GenericColor::LIGHT_BLUE, 500);
                }

            } else if (bandebot_twin_.currentBandebotState == BANDEBOT_APP_STATE::Serving) {

                setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayUpper, TRAYLIGHTS_MODE::On, GenericColor::LIGHT_BLUE, 500);

                // Todo: Change lower tray mode and colors every 5 seconds to make it more dynamic
                // setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayLower, TRAYLIGHTS_MODE::Chasser, GenericColor::LIGHT_BLUE, 25);
                setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayLower, TRAYLIGHTS_MODE::Dimmering, GenericColor::LIGHT_BLUE, 750);


            } else {

                setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayUpper, TRAYLIGHTS_MODE::Flashing, GenericColor::LIGHT_BLUE, 500);
                setDisplayTrayLight(LIGHTS_TYPE::DisplayTrayLower, TRAYLIGHTS_MODE::Flashing, GenericColor::LIGHT_BLUE, 500);

           }
            
            last_bandebot_app_state_ = bandebot_twin_.currentBandebotState;
            last_mobile_base_state_ = bandebot_twin_.currentMobileBaseState;
            RCLCPP_INFO(this->get_logger(), "BANDEBOT APP STATE changed to: %u", static_cast<unsigned int>(bandebot_twin_.currentBandebotState));
        }
    }

    void messageBandebotHardwareStatus()
    {
        auto ros_msg = rogue_droids_interfaces::msg::RobotHardwareStatus();

        // ACU
        if( bandebot_twin_.auxiliaryControlUnitPresent ) {

            if( bandebot_twin_.auxiliaryControlUnitStatus == BOARD_STATUS::NOT_CALIBRATED )
                ros_msg.acu_status = (uint8_t)HARDWARE_STATUS::DETECTED_NOT_CALIBRATED;
            else if( bandebot_twin_.auxiliaryControlUnitStatus == BOARD_STATUS::READY )
                ros_msg.acu_status = (uint8_t)HARDWARE_STATUS::DETECTED_READY;
            else
                ros_msg.acu_status = (uint8_t)HARDWARE_STATUS::UNKNOWN;

            if( (this->get_clock()->now() - bandebot_twin_.auxiliaryControlUnitLastUpdate).seconds() > 2.0 )
                ros_msg.acu_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.acu_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;

        // UCU
        if( bandebot_twin_.upperControlUnitPresent ) {

            if( bandebot_twin_.upperControlUnitStatus == BOARD_STATUS::NOT_CALIBRATED )
                ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::DETECTED_NOT_CALIBRATED;
            else if( bandebot_twin_.upperControlUnitStatus == BOARD_STATUS::READY )
                ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::DETECTED_READY;
            else
                ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::UNKNOWN;

            if( (this->get_clock()->now() - bandebot_twin_.upperControlUnitLastUpdate).seconds() > 2.0 )
                ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;
        
        // LCU
        if( bandebot_twin_.lightsControlUnitPresent ) {

            if( bandebot_twin_.lightsControlUnitStatus == BOARD_STATUS::NOT_CALIBRATED )
                ros_msg.lcu_status = (uint8_t)HARDWARE_STATUS::DETECTED_NOT_CALIBRATED;
            else if( bandebot_twin_.lightsControlUnitStatus == BOARD_STATUS::READY )
                ros_msg.lcu_status = (uint8_t)HARDWARE_STATUS::DETECTED_READY;
            else
                ros_msg.lcu_status = (uint8_t)HARDWARE_STATUS::UNKNOWN;

            if( (this->get_clock()->now() - bandebot_twin_.lightsControlUnitLastUpdate).seconds() > 2.0 )
                ros_msg.lcu_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.lcu_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;

        // Error counts
        ros_msg.acu_error_count = bandebot_twin_.auxiliaryControlUnitCanErrorCount;
        ros_msg.ucu_error_count = bandebot_twin_.upperControlUnitCanErrorCount;
        ros_msg.lcu_error_count = bandebot_twin_.lightsControlUnitCanErrorCount;

        robot_hardware_status_publisher_->publish(ros_msg);
    }
    
    void setSidelights(SIDELIGHTS_MODE mode, SIDELIGHTS_COLOR color, uint16_t period = 500)
    {
        if (!sidelights_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "mulita/set_sidelights service not available");
            return;
        }
        
        auto request = std::make_shared<rogue_droids_interfaces::srv::SetSidelights::Request>();
        request->mode = (uint8_t)mode;
        request->color = (uint8_t)color;
        request->period = period;
        
        // Send async request with callback - non-blocking
        sidelights_client_->async_send_request(request,
            [this](rclcpp::Client<rogue_droids_interfaces::srv::SetSidelights>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_DEBUG(this->get_logger(), "Sidelights updated successfully");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to set sidelights: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in sidelights service response: %s", e.what());
                }
            });
        
        RCLCPP_DEBUG(this->get_logger(), "Sidelights service call sent asynchronously");
    }

    void setDisplayTrayLight(LIGHTS_TYPE type, TRAYLIGHTS_MODE mode, GenericColor color, uint16_t period) {

        auto ros_msg = rogue_droids_interfaces::msg::CanFrame();

        ros_msg.len = 8;
        ros_msg.data[0] = (uint8_t)type;
        ros_msg.data[1] = (uint8_t)mode;
        ros_msg.data[2] = color.getRed();   // Red
        ros_msg.data[3] = color.getGreen(); // Green
        ros_msg.data[4] = color.getBlue();  // Blue
        ros_msg.data[5] = period & 0xFF; // LSB of period
        ros_msg.data[6] = (period >> 8) & 0xFF; // MSB of period

        display_tray_lights_mode_publisher_->publish(ros_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PromobotNode>());
    rclcpp::shutdown();
    return 0;
}