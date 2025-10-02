/*****************************************************************************
 * 
 * Copyright 2025 Roman Di Lullo
 * 
 * 
 * Created: 23/04/2025
 * Last Modified: 11/09/2025
 * 
 *****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "rogue_droids_interfaces/msg/can_frame.hpp"
#include "rogue_droids_interfaces/msg/robot_hardware_status.hpp"
#include "rogue_droids_interfaces/srv/calibrate.hpp"
#include "rogue_droids_interfaces/srv/start_loading.hpp"
#include "rogue_droids_interfaces/srv/stop_loading.hpp"
#include "rogue_droids_interfaces/srv/start_unloading.hpp"
#include "rogue_droids_interfaces/srv/stop_unloading.hpp"
#include "rogue_droids_interfaces/srv/start_serving.hpp"
#include "rogue_droids_interfaces/srv/stop_serving.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "../../mulita/include/mulita_defines.h"
#include "../../mulita/include/bandebot_defines.h"

#include "bandebot_app/bandebot_digital_twin.h"

class BandebotNode : public rclcpp::Node
{
public:
    BandebotNode() : Node("bandebot")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Bandebot Node...");

        // Initialize the node
        bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::WaitingMobileBaseReady;

        // Create publishers
        bandebot_app_state_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("bandebot/app_state", 10);
        requested_tray_position_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("bandebot/requested_tray_position", 10);
        tray_illumination_requested_state_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("bandebot/tray_illumination_requested_state", 10);
        robot_hardware_status_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::RobotHardwareStatus>("bandebot/robot_hardware_status", 10);
        power_relays_control_publisher_ = this->create_publisher<rogue_droids_interfaces::msg::CanFrame>("mulita/power_relays_control", 10);

        // Create subscribers

        mulita_robot_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/mobile_base_state", 10, std::bind(&BandebotNode::on_mulita_robot_state, this, std::placeholders::_1));
            
        tray_content_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/tray_content_state", 10, std::bind(&BandebotNode::on_tray_content_state, this, std::placeholders::_1));
    
        current_tray_position_1_4_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/current_tray_position_1_4", 10, std::bind(&BandebotNode::on_current_tray_position_1_4, this, std::placeholders::_1));
    
        current_tray_position_5_8_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/current_tray_position_5_8", 10, std::bind(&BandebotNode::on_current_tray_position_5_8, this, std::placeholders::_1));
    
        tray_illumination_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/tray_illumination_state", 10, std::bind(&BandebotNode::on_tray_illumination_state, this, std::placeholders::_1));
    
        tray_measured_distance_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "bandebot/tray_measured_distance", 10, std::bind(&BandebotNode::on_tray_measured_distance, this, std::placeholders::_1));
    
        power_supply_state_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/power_supply_state", 10, std::bind(&BandebotNode::on_power_supply_state, this, std::placeholders::_1));
    
        hw_id_information_subscriber_ = this->create_subscription<rogue_droids_interfaces::msg::CanFrame>(
            "mulita/hw_id_information", 10, std::bind(&BandebotNode::on_hw_id_information, this, std::placeholders::_1));


        // Create services
        srv_calibrate_ = this->create_service<rogue_droids_interfaces::srv::Calibrate>(
            std::string("bandebot/calibrate"),
            std::bind(&BandebotNode::handle_srv_calibrate, this, std::placeholders::_1, std::placeholders::_2));

        srv_start_loading_ = this->create_service<rogue_droids_interfaces::srv::StartLoading>(
            std::string("bandebot/start_loading"),
            std::bind(&BandebotNode::handle_srv_start_loading_, this, std::placeholders::_1, std::placeholders::_2));

        srv_stop_loading_ = this->create_service<rogue_droids_interfaces::srv::StopLoading>(
            std::string("bandebot/stop_loading"),
            std::bind(&BandebotNode::handle_srv_stop_loading_, this, std::placeholders::_1, std::placeholders::_2));

        srv_start_unloading_ = this->create_service<rogue_droids_interfaces::srv::StartUnloading>(
            std::string("bandebot/start_unloading"),
            std::bind(&BandebotNode::handle_srv_start_unloading_, this, std::placeholders::_1, std::placeholders::_2));

        srv_stop_unloading_ = this->create_service<rogue_droids_interfaces::srv::StopUnloading>(
            std::string("bandebot/stop_unloading"),
            std::bind(&BandebotNode::handle_srv_stop_unloading_, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_start_serving_ = this->create_service<rogue_droids_interfaces::srv::StartServing>(
            std::string("bandebot/start_serving"),
            std::bind(&BandebotNode::handle_srv_start_serving_, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_serving_ = this->create_service<rogue_droids_interfaces::srv::StopServing>(
            std::string("bandebot/stop_serving"),
            std::bind(&BandebotNode::handle_srv_stop_serving_, this, std::placeholders::_1, std::placeholders::_2));
        
        operation_start_time_ = rclcpp::Clock().now();

        messageBandebotAppState();

        // Create a timer for the main loop function
        main_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&BandebotNode::mainLoopFunction, this));

        robot_state_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&BandebotNode::messageBandebotAppState, this));

        bandebot_hardware_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000),
            std::bind(&BandebotNode::messageBandebotHardwareStatus, this));

        RCLCPP_INFO(this->get_logger(), "Bandebot node has been started.");
    }

private:

    BandebotTwin bandebot_twin_;
    BANDEBOT_APP_STATE last_bandebot_app_state_  = BANDEBOT_APP_STATE::Unknown;
    rclcpp::Time operation_start_time_;

    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr robot_state_update_timer_;
    rclcpp::TimerBase::SharedPtr bandebot_hardware_update_timer_;

    rclcpp::Service<rogue_droids_interfaces::srv::Calibrate>::SharedPtr srv_calibrate_;
    rclcpp::Service<rogue_droids_interfaces::srv::StartLoading>::SharedPtr srv_start_loading_;
    rclcpp::Service<rogue_droids_interfaces::srv::StopLoading>::SharedPtr srv_stop_loading_;
    rclcpp::Service<rogue_droids_interfaces::srv::StartUnloading>::SharedPtr srv_start_unloading_; 
    rclcpp::Service<rogue_droids_interfaces::srv::StopUnloading>::SharedPtr srv_stop_unloading_;
    rclcpp::Service<rogue_droids_interfaces::srv::StartServing>::SharedPtr srv_start_serving_;
    rclcpp::Service<rogue_droids_interfaces::srv::StopServing>::SharedPtr srv_stop_serving_;

    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr bandebot_app_state_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr requested_tray_position_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr tray_illumination_requested_state_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::RobotHardwareStatus>::SharedPtr robot_hardware_status_publisher_;
    rclcpp::Publisher<rogue_droids_interfaces::msg::CanFrame>::SharedPtr power_relays_control_publisher_;

    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr mulita_robot_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr tray_content_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr current_tray_position_1_4_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr current_tray_position_5_8_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr tray_illumination_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr tray_measured_distance_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr power_supply_state_subscriber_;
    rclcpp::Subscription<rogue_droids_interfaces::msg::CanFrame>::SharedPtr hw_id_information_subscriber_;


    void mainLoopFunction()
    {
        auto elapsed_time = rclcpp::Clock().now() - operation_start_time_;

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
                operation_start_time_ = rclcpp::Clock().now();                

                break;

            case BANDEBOT_APP_STATE::IdentifyingHardware:

                // UpdateSidelightsMode(SIDELIGHTS_MODE::Flashing, SIDELIGHTS_COLOR::Orange);

                if( bandebot_twin_.IsApplicationHwReady() )
                {
                    bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ApplicationReady;
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

                // TODO: ??
                break;

            //---------------------------

            case BANDEBOT_APP_STATE::ApplicationReady:
                // UpdateSidelightsMode(SIDELIGHTS_MODE::Off);
                allTrayLEDsOff();
                break;

            case BANDEBOT_APP_STATE::CalibratingHardware:

                // UpdateSidelightsMode(SIDELIGHTS_MODE::Dimmering, SIDELIGHTS_COLOR::Blue);

                // Verificar si estan todos los elavadores listos
                if( bandebot_twin_.IsApplicationCalibrated() )
                {
                    bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ReadyUnloaded;
                }
                else
                {
                    if( elapsed_time.seconds() > CALIBRATION_TIMEOUT_SEC )
                    {
                        RCLCPP_INFO(this->get_logger(), "Calibration timedout after %f seconds", elapsed_time.seconds());
                        bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ApplicationReady;
                    }
                }
                break;


            case BANDEBOT_APP_STATE::ReadyUnloaded:
                allTrayLEDsOff();
                break;

            case BANDEBOT_APP_STATE::Loading:

                allTrayLEDsOn();

                /* 
                    For each elevator, check if the tray is occupied and how long it has been occupied.
                    If the tray is occupied for more than 5 seconds and tray position is not the top one (position 1),
                    then move the tray to the next position (current position minus one)
                */
                for(uint8_t i = 0; i < APP_ELEVATOR_COUNT; i++) {

                    if( bandebot_twin_.elevators[i].GetTrayPosition() > TOP_TRAY_POSITION ) {

                        if( bandebot_twin_.elevators[i].trayOccupiedFor(rclcpp::Clock().now(), LOAD_UNLOAD_TRAY_NO_CHANGE_TIMEOUT_SEC) ) {
                                requestTrayPosition(i + 1, bandebot_twin_.elevators[i].GetTrayPosition() - 1); 
                            }
                        }
                    }

                if( bandebot_twin_.CheckIfAllTraysFull() ) {
                    bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ReadyLoaded;
                }
                break;

            case BANDEBOT_APP_STATE::Unloading:

                allTrayLEDsOn();

                /* 
                    For each elevator, check if the tray is empty and how long it has been empty.
                    If the tray is empty for more than 5 seconds and tray position is not the bottom one (position 11),
                    then move the tray to the next position (current position plus one)
                */
            for(uint8_t i = 0; i < APP_ELEVATOR_COUNT; i++) {

                    if( bandebot_twin_.elevators[i].GetTrayPosition() < BOTTOM_TRAY_POSITION ) {

                        if( bandebot_twin_.elevators[i].trayEmptyFor(rclcpp::Clock().now(), LOAD_UNLOAD_TRAY_NO_CHANGE_TIMEOUT_SEC) ) {
                                requestTrayPosition(i + 1, bandebot_twin_.elevators[i].GetTrayPosition() + 1); 
                            }
                        }
                    }

                if( bandebot_twin_.CheckIfAllTraysEmpty() ) {
                    bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::ApplicationReady;
                }
                break;

            case BANDEBOT_APP_STATE::ReadyLoaded:

                allTrayLEDsOff();
                break;


            case BANDEBOT_APP_STATE::Serving:

                if( bandebot_twin_.currentMobileBaseState == MULITA_ROBOT_STATE::ReadyMoving ) {

                    allTrayLEDsOff();

                } else {

                    bool trayLedState;

                    for(uint8_t i = 0; i < APP_ELEVATOR_COUNT; i++) {

                        trayLedState = false;

                        if( bandebot_twin_.elevators[i].GetPositionState() == CURRENT_TRAY_POSITION_STATE::Stopped) {

                            switch( bandebot_twin_.elevators[i].GetContentState())
                            {
                                case TRAY_CONTENT_STATE::Occupied:
                                case TRAY_CONTENT_STATE::MovementDetected:
                                    // LED ON
                                    trayLedState = true;
                                    break;

                                case TRAY_CONTENT_STATE::Empty:
                                    // LED OFF

                                    // If the tray is empty and the position is not the bottom one, request to move it down
                                    if( bandebot_twin_.elevators[i].GetTrayPosition() < BOTTOM_TRAY_POSITION ) {
                                        if( bandebot_twin_.elevators[i].trayEmptyFor(rclcpp::Clock().now(), SERVING_TRAY_NO_CHANGE_TIMEOUT_SEC) ) {
                                                requestTrayPosition(i + 1, bandebot_twin_.elevators[i].GetTrayPosition() + 1); 
                                            }
                                        }
                                    break;

                                default:
                                    break;
                            }

                        } else {
                            // LED OFF
                        }

                        requestTrayIlumination(i+1, trayLedState);
                    }
                
                    if( bandebot_twin_.CheckIfAllTraysEmpty() ) {
                        bandebot_twin_.currentBandebotState = BANDEBOT_APP_STATE::Unloading;
                    }                   

                }

                break;

            default:
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


    void on_tray_content_state(const rogue_droids_interfaces::msg::CanFrame msg) {

        // [TRAY_CONTENT_STATE_1], [TRAY_CONTENT_STATE_2], ... [TRAY_CONTENT_STATE_8]
        if(msg.len == 8)
            bandebot_twin_.ProcessTrayContentState(rclcpp::Clock().now(), msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

    }
    
    void on_current_tray_position_1_4(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {tray_1_current_pos},[CURRENT_TRAY_POSITION_STATE_1], , ... {tray_4_current_pos},[CURRENT_TRAY_POSITION_STATE_4] 
        if( msg.len == 8)
            bandebot_twin_.ProcessCurrentTrayPositionState_1_4(rclcpp::Clock().now(), msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        
    }

    void on_current_tray_position_5_8(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {tray_5_current_pos},[CURRENT_TRAY_POSITION_STATE_5], , ... {tray_8_current_pos},[CURRENT_TRAY_POSITION_STATE_8]
        if( msg.len == 8)
            bandebot_twin_.ProcessCurrentTrayPositionState_5_8(rclcpp::Clock().now(), msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        
    }

    void on_tray_illumination_state(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {ilum_state_tray_1 ON(255)/OFF(0)}, ...
        if( msg.len == 8)
            bandebot_twin_.ProcessTrayIlluminationState(msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    }

    void on_tray_measured_distance(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {distance_tray_1_mm}, {distance_tray_2_mm}, ... {distance_tray_8_mm}
        if( msg.len == 8)
            bandebot_twin_.ProcessTrayMeasuredDistance(rclcpp::Clock().now(), msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    }

    void on_power_supply_state(const rogue_droids_interfaces::msg::CanFrame msg) {

        // {battery_voltage}, {current_ucu}, {current_ecus}, {current_motor}
        if( msg.len == 8)
            bandebot_twin_.ProcessPowerSupplyState(rclcpp::Clock().now(), uint16_t ((msg.data[1] << 8) | msg.data[0]),
                int16_t ((msg.data[3] << 8) | msg.data[2]), int16_t ((msg.data[5] << 8) | msg.data[4]),
                int16_t ((msg.data[7] << 8) | msg.data[6]));
    }

    void on_hw_id_information(const rogue_droids_interfaces::msg::CanFrame msg) {

        if( msg.len >= 4) {
            uint16_t error_count = static_cast<uint16_t>(msg.data[2]) | (static_cast<uint16_t>(msg.data[3]) << 8);
            bandebot_twin_.ProcessHwInformation(rclcpp::Clock().now(), msg.data[0], msg.data[1], error_count);
        }
    }

    void handle_srv_calibrate(
        const std::shared_ptr<rogue_droids_interfaces::srv::Calibrate::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::Calibrate::Response> response)
    {
        response->success = bandebot_twin_.StartCalibration(request->mode);
        operation_start_time_ = rclcpp::Clock().now();
    }

    void handle_srv_start_loading_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StartLoading::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StartLoading::Response> response)
    {
        response->success = bandebot_twin_.StartLoading(request->mode);
    }
    
    void handle_srv_stop_loading_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopLoading::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopLoading::Response> response)
    {
        response->success = bandebot_twin_.StopLoading(request->mode);
    }
    
    void handle_srv_start_unloading_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StartUnloading::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StartUnloading::Response> response)
    {
        response->success = bandebot_twin_.StartUnloading(request->mode);
    }
    
    void handle_srv_stop_unloading_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopUnloading::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopUnloading::Response> response)
    {
        response->success = bandebot_twin_.StopUnloading(request->mode);
    }
    
    void handle_srv_start_serving_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StartServing::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StartServing::Response> response)
    {
        response->success = bandebot_twin_.StartServing(request->mode);
    }
    
    void handle_srv_stop_serving_(
        const std::shared_ptr<rogue_droids_interfaces::srv::StopServing::Request> request,
        std::shared_ptr<rogue_droids_interfaces::srv::StopServing::Response> response)
    {
        response->success = bandebot_twin_.StopServing(request->mode);
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

        if(bandebot_twin_.currentBandebotState != last_bandebot_app_state_)
        {
            last_bandebot_app_state_ = bandebot_twin_.currentBandebotState;
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

            if( (rclcpp::Clock().now() - bandebot_twin_.auxiliaryControlUnitLastUpdate).seconds() > 2.0 )
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

            if( (rclcpp::Clock().now() - bandebot_twin_.upperControlUnitLastUpdate).seconds() > 2.0 )
                ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.ucu_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;
            
        // ECU 1
        if( bandebot_twin_.elevatorControlUnit1Present ) {

            if( bandebot_twin_.elevatorControlUnit1Status == BOARD_STATUS::NOT_CALIBRATED )
                ros_msg.ecu_1_status = (uint8_t)HARDWARE_STATUS::DETECTED_NOT_CALIBRATED;
            else if( bandebot_twin_.elevatorControlUnit1Status == BOARD_STATUS::READY )
                ros_msg.ecu_1_status = (uint8_t)HARDWARE_STATUS::DETECTED_READY;
            else
                ros_msg.ecu_1_status = (uint8_t)HARDWARE_STATUS::UNKNOWN;

            if( (rclcpp::Clock().now() - bandebot_twin_.elevatorControlUnit1LastUpdate).seconds() > 2.0 )
                ros_msg.ecu_1_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.ecu_1_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;

        // ECU 2
        if( bandebot_twin_.elevatorControlUnit2Present ) {

            if( bandebot_twin_.elevatorControlUnit2Status == BOARD_STATUS::NOT_CALIBRATED )
                ros_msg.ecu_2_status = (uint8_t)HARDWARE_STATUS::DETECTED_NOT_CALIBRATED;
            else if( bandebot_twin_.elevatorControlUnit2Status == BOARD_STATUS::READY )
                ros_msg.ecu_2_status = (uint8_t)HARDWARE_STATUS::DETECTED_READY;
            else
                ros_msg.ecu_2_status = (uint8_t)HARDWARE_STATUS::UNKNOWN;

            if( (rclcpp::Clock().now() - bandebot_twin_.elevatorControlUnit2LastUpdate).seconds() > 2.0 )
                ros_msg.ecu_2_status = (uint8_t)HARDWARE_STATUS::DETECTED_LOST_COMMS;

        } else
            ros_msg.ecu_2_status = (uint8_t)HARDWARE_STATUS::NOT_DETECTED;


        // Error counts
        ros_msg.acu_error_count = bandebot_twin_.auxiliaryControlUnitCanErrorCount;
        ros_msg.ucu_error_count = bandebot_twin_.upperControlUnitCanErrorCount;
        ros_msg.ecu_1_error_count = bandebot_twin_.elevatorControlUnit1CanErrorCount;
        ros_msg.ecu_2_error_count = bandebot_twin_.elevatorControlUnit2CanErrorCount;

        robot_hardware_status_publisher_->publish(ros_msg);
    }
    
    void requestTrayIlumination(uint8_t tray_id, bool requested_state)
    {
        if(tray_id >= 1 || tray_id <= APP_ELEVATOR_COUNT) {

            if( bandebot_twin_.elevators[tray_id - 1].GetIlluminationState() != requested_state ) {

                auto ros_msg = rogue_droids_interfaces::msg::CanFrame();

                ros_msg.len = 2;
                ros_msg.data[0] = tray_id;
                ros_msg.data[1] = requested_state ? 255 : 0;     // Illumination state (0 for OFF, 255 for ON)

                tray_illumination_requested_state_publisher_->publish(ros_msg);
            }            
        }
    }

    void allTrayLEDsOn()
    {
        for(uint8_t i = 0; i < APP_ELEVATOR_COUNT; i++)
            requestTrayIlumination(i+1, true);
    }

    void allTrayLEDsOff()
    {
        for(uint8_t i = 0; i < APP_ELEVATOR_COUNT; i++)
            requestTrayIlumination(i+1, false);
    }

    void requestTrayPosition(uint8_t tray_id, uint8_t requested_position)
    {
        if(tray_id >= 1 || tray_id <= APP_ELEVATOR_COUNT) {

            if( bandebot_twin_.elevators[tray_id - 1].GetTrayPosition() != requested_position ) {

                auto ros_msg = rogue_droids_interfaces::msg::CanFrame();

                ros_msg.len = 2;
                ros_msg.data[0] = tray_id;
                ros_msg.data[1] = requested_position;

                requested_tray_position_publisher_->publish(ros_msg);
            }            
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BandebotNode>());
    rclcpp::shutdown();
    return 0;
}