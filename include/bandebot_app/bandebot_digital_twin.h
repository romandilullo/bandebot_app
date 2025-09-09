#ifndef BANDEBOT_DIGITAL_TWIN_H
#define BANDEBOT_DIGITAL_TWIN_H

#include <rclcpp/rclcpp.hpp> // Include rclcpp for rclcpp::Time

#include "../../mulita/include/mulita_defines.h"
#include "../../mulita/include/bandebot_defines.h"

#define BOTTOM_TRAY_POSITION 11
#define TOP_TRAY_POSITION 1

class ElevatorTwin
{
public:
    ElevatorTwin() {
        
    }

    uint8_t GetTrayPosition() const {
        return trayPosition;
    }
    void SetTrayPosition(uint8_t position) {
        trayPosition = position;
    }
    CURRENT_TRAY_POSITION_STATE GetPositionState() const {
        return positionState;
    };
    void SetPositionState(uint8_t state) {
        positionState = static_cast<CURRENT_TRAY_POSITION_STATE>(state);
    }
    TRAY_CONTENT_STATE GetContentState() const {
        return contentState;
    }
    rclcpp::Time GetContentStateLastChange() const {
        return contentStateLastChange;
    }
    void SetContentState(uint8_t new_state, const rclcpp::Time& time) {
        if(contentState != (TRAY_CONTENT_STATE)new_state)
            contentStateLastChange = time;
        contentState = (TRAY_CONTENT_STATE)new_state;
    }
    uint16_t GetDistanceMm() const {
        return distanceMm;
    };
    void SetDistanceMm(uint8_t distance) {
        distanceMm = distance;
    };
    bool GetIlluminationState() const {
        return illuminationState;
    };
    void SetIlluminationState(bool state) {
        illuminationState = state;
    };

    bool trayOccupiedFor(const rclcpp::Time& time, uint16_t seconds) const {

        return (contentState == TRAY_CONTENT_STATE::Occupied) &&
               (time - contentStateLastChange).seconds() >= seconds;
    }

    bool trayEmptyFor(const rclcpp::Time& time, uint16_t seconds) const {

        return (contentState == TRAY_CONTENT_STATE::Empty) &&
               (time - contentStateLastChange).seconds() >= seconds;
    }

private:
    uint8_t trayPosition = 0;
    CURRENT_TRAY_POSITION_STATE positionState = CURRENT_TRAY_POSITION_STATE::Unknown;
    TRAY_CONTENT_STATE contentState = TRAY_CONTENT_STATE::Unknown;
    rclcpp::Time contentStateLastChange;
    uint8_t distanceMm = 0;
    bool illuminationState = false;
};


class BandebotTwin
{
public:
    BandebotTwin();
    void ProcessHwInformation(rclcpp::Time time_stamp, uint8_t board_id, uint8_t board_status, uint16_t can_error_count);

    void ProcessTrayContentState(rclcpp::Time time_stamp, uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
                                 uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8);

    void ProcessCurrentTrayPositionState_1_4(rclcpp::Time time_stamp, uint8_t tray_1_pos, uint8_t tray_1_state,
                                             uint8_t tray_2_pos, uint8_t tray_2_state,
                                             uint8_t tray_3_pos, uint8_t tray_3_state,
                                             uint8_t tray_4_pos, uint8_t tray_4_state);

    void ProcessCurrentTrayPositionState_5_8(rclcpp::Time time_stamp, uint8_t tray_5_pos, uint8_t tray_5_state,
                                             uint8_t tray_6_pos, uint8_t tray_6_state,
                                             uint8_t tray_7_pos, uint8_t tray_7_state,
                                             uint8_t tray_8_pos, uint8_t tray_8_state);

    void ProcessTrayIlluminationState(uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
                                      uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8);

    void ProcessTrayMeasuredDistance(rclcpp::Time time_stamp, uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
                                     uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8);

    void ProcessPowerSupplyState(rclcpp::Time time_stamp, uint16_t batteryVoltage, int16_t UCUCurrent, 
                                 int16_t ECUCurrent, int16_t MotorCurrent);

    bool IsApplicationHwReady();
    bool IsApplicationCalibrated();
    bool CheckIfAllTraysEmpty();
    bool CheckIfAllTraysFull();
    bool SetSidelightsMode(SIDELIGHTS_MODE mode, SIDELIGHTS_COLOR color);
    
    bool StartCalibration(uint16_t mode);
    bool StartLoading(uint16_t mode);
    bool StopLoading(uint16_t mode);
    bool StartUnloading(uint16_t mode);
    bool StopUnloading(uint16_t mode);
    bool StartServing(uint16_t mode);
    bool StopServing(uint16_t mode);
  
    bool auxiliaryControlUnitPresent       = false;
    BOARD_STATUS auxiliaryControlUnitStatus = BOARD_STATUS::UNKNOWN;
    rclcpp::Time auxiliaryControlUnitLastUpdate;
    int16_t auxiliaryControlUnitCanErrorCount = 0;

    bool upperControlUnitPresent           = false;
    BOARD_STATUS upperControlUnitStatus = BOARD_STATUS::UNKNOWN;
    rclcpp::Time upperControlUnitLastUpdate;
    int16_t upperControlUnitCanErrorCount = 0;

    bool elevatorControlUnit1Present       = false;
    BOARD_STATUS elevatorControlUnit1Status = BOARD_STATUS::UNKNOWN;
    rclcpp::Time elevatorControlUnit1LastUpdate;
    int16_t elevatorControlUnit1CanErrorCount = 0;

    bool elevatorControlUnit2Present       = false;
    BOARD_STATUS elevatorControlUnit2Status = BOARD_STATUS::UNKNOWN;
    rclcpp::Time elevatorControlUnit2LastUpdate;
    int16_t elevatorControlUnit2CanErrorCount = 0;


    MULITA_ROBOT_STATE currentMobileBaseState  = MULITA_ROBOT_STATE::Unknown;
    BANDEBOT_APP_STATE currentBandebotState = BANDEBOT_APP_STATE::Unknown;

    MULITA_MOVING_STATE currentMovingState = MULITA_MOVING_STATE::Unknown;
    SIDELIGHTS_MODE currentSidelightsMode  = SIDELIGHTS_MODE::Unknown;
    SIDELIGHTS_COLOR currentSidelightsColor = SIDELIGHTS_COLOR::Amber;

    float batteryVoltage = 0.0f;
    float UCUCurrent = 0.0f;
    float ECUCurrent = 0.0f;
    float motorCurrent = 0.0f;

    rclcpp::Time batteryVoltageLastUpdate;

    ElevatorTwin elevators[8];
};

#endif // BANDEBOT_DIGITAL_TWIN_H
