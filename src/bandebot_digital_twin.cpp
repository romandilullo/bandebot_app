/*****************************************************************************
 * 
 * Copyright 2025 Roman Di Lullo
 * 
 * 
 * Created: 05/05/2025
 * Last Modified: 28/10/2025
 * 
 *****************************************************************************/

#include "bandebot_app/bandebot_digital_twin.h"

BandebotTwin::BandebotTwin() {

}

void BandebotTwin::ProcessHwInformation(rclcpp::Time time_stamp, uint8_t board_id, uint8_t board_status, uint16_t can_error_count) {

    switch (static_cast<BOARD_ID>(board_id))
    {
        case BOARD_ID::ACU:
            auxiliaryControlUnitPresent = true;
            auxiliaryControlUnitStatus = (BOARD_STATUS)board_status;
            auxiliaryControlUnitCanErrorCount = can_error_count;
            auxiliaryControlUnitLastUpdate = time_stamp;
            break;
        case BOARD_ID::UCU:
            upperControlUnitPresent = true;
            upperControlUnitStatus = (BOARD_STATUS)board_status;
            upperControlUnitCanErrorCount = can_error_count;
            upperControlUnitLastUpdate = time_stamp;
            break;
        case BOARD_ID::ECU_1:
            elevatorControlUnit1Present = true;
            elevatorControlUnit1Status = (BOARD_STATUS)board_status;
            elevatorControlUnit1CanErrorCount = can_error_count;
            elevatorControlUnit1LastUpdate = time_stamp;
            break;
        case BOARD_ID::ECU_2:
            elevatorControlUnit2Present = true;
            elevatorControlUnit2Status = (BOARD_STATUS)board_status;
            elevatorControlUnit2CanErrorCount = can_error_count;
            elevatorControlUnit2LastUpdate = time_stamp;
            break;
        default:
            break;
    }
}

bool BandebotTwin::ProcessTrayContentState(rclcpp::Time time_stamp, uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
                                            uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8) {

    bool result = false;
    
    result |= elevators[0].SetContentState(tray_1, time_stamp);
    result |= elevators[1].SetContentState(tray_2, time_stamp);
    result |= elevators[2].SetContentState(tray_3, time_stamp);
    result |= elevators[3].SetContentState(tray_4, time_stamp);
    result |= elevators[4].SetContentState(tray_5, time_stamp);
    result |= elevators[5].SetContentState(tray_6, time_stamp);
    result |= elevators[6].SetContentState(tray_7, time_stamp);
    result |= elevators[7].SetContentState(tray_8, time_stamp);

    return result;
}

void BandebotTwin::ProcessCurrentTrayPositionState_1_4(rclcpp::Time time_stamp, uint8_t tray_1_pos, uint8_t tray_1_state,
                                                        uint8_t tray_2_pos, uint8_t tray_2_state,
                                                        uint8_t tray_3_pos, uint8_t tray_3_state,
                                                        uint8_t tray_4_pos, uint8_t tray_4_state) {

    elevators[0].SetPositionState(tray_1_state);
    elevators[0].SetTrayPosition(tray_1_pos);
    elevators[1].SetPositionState(tray_2_state);
    elevators[1].SetTrayPosition(tray_2_pos);
    elevators[2].SetPositionState(tray_3_state);
    elevators[2].SetTrayPosition(tray_3_pos);
    elevators[3].SetPositionState(tray_4_state);
    elevators[3].SetTrayPosition(tray_4_pos);

    time_stamp = time_stamp; // TODO: implement
}

void BandebotTwin::ProcessCurrentTrayPositionState_5_8(rclcpp::Time time_stamp, uint8_t tray_5_pos, uint8_t tray_5_state,
                                                        uint8_t tray_6_pos, uint8_t tray_6_state,
                                                        uint8_t tray_7_pos, uint8_t tray_7_state,
                                                        uint8_t tray_8_pos, uint8_t tray_8_state) {

    elevators[4].SetPositionState(tray_5_state);
    elevators[4].SetTrayPosition(tray_5_pos);
    elevators[5].SetPositionState(tray_6_state);
    elevators[5].SetTrayPosition(tray_6_pos);
    elevators[6].SetPositionState(tray_7_state);
    elevators[6].SetTrayPosition(tray_7_pos);
    elevators[7].SetPositionState(tray_8_state);
    elevators[7].SetTrayPosition(tray_8_pos);

    time_stamp = time_stamp; // TODO: implement
}

void BandebotTwin::ProcessTrayIlluminationState(uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
    uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8) {

    elevators[0].SetIlluminationState(tray_1 != 0);
    elevators[1].SetIlluminationState(tray_2 != 0);
    elevators[2].SetIlluminationState(tray_3 != 0);
    elevators[3].SetIlluminationState(tray_4 != 0);
    elevators[4].SetIlluminationState(tray_5 != 0);
    elevators[5].SetIlluminationState(tray_6 != 0);
    elevators[6].SetIlluminationState(tray_7 != 0);
    elevators[7].SetIlluminationState(tray_8 != 0);
}

void BandebotTwin::ProcessTrayMeasuredDistance(rclcpp::Time time_stamp, uint8_t tray_1, uint8_t tray_2, uint8_t tray_3, uint8_t tray_4,
    uint8_t tray_5, uint8_t tray_6, uint8_t tray_7, uint8_t tray_8) {

    elevators[0].SetDistanceMm(tray_1);
    elevators[1].SetDistanceMm(tray_2);
    elevators[2].SetDistanceMm(tray_3);
    elevators[3].SetDistanceMm(tray_4);
    elevators[4].SetDistanceMm(tray_5);
    elevators[5].SetDistanceMm(tray_6);
    elevators[6].SetDistanceMm(tray_7);
    elevators[7].SetDistanceMm(tray_8);

    time_stamp = time_stamp; // TODO: implement
}


void BandebotTwin::ProcessPowerSupplyState(rclcpp::Time time_stamp, uint16_t battery_voltage, int16_t UCU_current, 
                                             int16_t ECU_current, int16_t Motor_current) {

    batteryVoltage = static_cast<float>(battery_voltage) / 1000.0f;
    UCUCurrent = static_cast<float>(UCU_current) / 1.0f;
    ECUCurrent = static_cast<float>(ECU_current) / 1.0f;
    motorCurrent = static_cast<float>(Motor_current) / 1.0f;

    batteryVoltageLastUpdate = time_stamp;
}


bool BandebotTwin::IsApplicationHwReady() {

    return ( upperControlUnitPresent && elevatorControlUnit1Present && elevatorControlUnit2Present);
}

bool BandebotTwin::IsApplicationCalibrated() {

    // Check if all present hardware is calibrated
    if( auxiliaryControlUnitPresent && (auxiliaryControlUnitStatus != BOARD_STATUS::READY) ) {
        return false;
    }
    if( upperControlUnitPresent && (upperControlUnitStatus != BOARD_STATUS::READY) ) {
        return false;
    }
    if( elevatorControlUnit1Present && (elevatorControlUnit1Status != BOARD_STATUS::READY) ) {
        return false;
    }
    if( elevatorControlUnit2Present && (elevatorControlUnit2Status != BOARD_STATUS::READY) ) {
        return false;
    }
    return true;
}


bool BandebotTwin::CheckIfAllTraysEmpty() {

    bool allEmpty = true;

    for (int index = 0; index < APP_ELEVATOR_COUNT; index++) {    
        
        if( (elevators[index].GetTrayPosition() != BOTTOM_TRAY_POSITION) ||
            (elevators[index].GetPositionState() != CURRENT_TRAY_POSITION_STATE::Stopped) ||
            (elevators[index].GetContentState() != TRAY_CONTENT_STATE::Empty)) {

            allEmpty = false;
            break;
        }
    }
    return allEmpty;
}


bool BandebotTwin::CheckIfAllTraysFull() {

    bool allFull = true;

    for (int index = 0; index < APP_ELEVATOR_COUNT; index++) {    
        
        if( (elevators[index].GetTrayPosition() != TOP_TRAY_POSITION) ||
            (elevators[index].GetPositionState() != CURRENT_TRAY_POSITION_STATE::Stopped) ||
            (elevators[index].GetContentState() != TRAY_CONTENT_STATE::Occupied)) {

                allFull = false;
            break;
        }
    }
    return allFull;
}


bool BandebotTwin::StartCalibration(uint16_t mode) {

    mode = mode;

    if( currentMobileBaseState != MULITA_ROBOT_STATE::Error &&

        ((currentBandebotState == BANDEBOT_APP_STATE::ApplicationReady) ||
         (currentBandebotState == BANDEBOT_APP_STATE::ReadyUnloaded)) ) {

        // Reset all hardware status
        auxiliaryControlUnitStatus = BOARD_STATUS::UNKNOWN;
        upperControlUnitStatus = BOARD_STATUS::UNKNOWN;
        elevatorControlUnit1Status = BOARD_STATUS::UNKNOWN;
        elevatorControlUnit2Status = BOARD_STATUS::UNKNOWN;

        currentBandebotState = BANDEBOT_APP_STATE::CalibratingHardware;
        return true;

    }

    return false;
}

bool BandebotTwin::StartLoading(uint16_t mode) {

    mode = mode;

    if((currentBandebotState == BANDEBOT_APP_STATE::ReadyUnloaded) ||
       (currentBandebotState == BANDEBOT_APP_STATE::ReadyLoaded))
    {
        currentBandebotState = BANDEBOT_APP_STATE::Loading;
        return true;
    }
    return false;
}

bool BandebotTwin::StopLoading(uint16_t mode) {

    mode = mode;

    if( currentBandebotState == BANDEBOT_APP_STATE::Loading )
    {
        if( CheckIfAllTraysEmpty() )
        {
            currentBandebotState = BANDEBOT_APP_STATE::ReadyUnloaded;
        } else {
            currentBandebotState = BANDEBOT_APP_STATE::ReadyLoaded;
        }

        return true;
    }

    return false;
}

bool BandebotTwin::StartUnloading(uint16_t mode) {

    mode = mode;

    if( (currentBandebotState == BANDEBOT_APP_STATE::Loading) || 
        (currentBandebotState == BANDEBOT_APP_STATE::ReadyLoaded))
    {
        currentBandebotState = BANDEBOT_APP_STATE::Unloading;
        return true;
    }

    return false;
}

bool BandebotTwin::StopUnloading(uint16_t mode) {

    mode = mode;

    if( currentBandebotState == BANDEBOT_APP_STATE::Unloading )
    {
        currentBandebotState = BANDEBOT_APP_STATE::Loading;
        return true;
    }

    return false;
}

bool BandebotTwin::StartServing(uint16_t mode) {

    mode = mode;

    if( currentBandebotState == BANDEBOT_APP_STATE::ReadyLoaded )
    {
        currentBandebotState = BANDEBOT_APP_STATE::Serving;
        return true;
    }

    return false;
}

bool BandebotTwin::StopServing(uint16_t mode) {

    mode = mode;

    if( currentBandebotState == BANDEBOT_APP_STATE::Serving )
    {
        currentBandebotState = BANDEBOT_APP_STATE::ReadyLoaded;
        return true;
    }

    return false;
}

bool BandebotTwin::SetSidelightsMode(SIDELIGHTS_MODE mode, SIDELIGHTS_COLOR color) {

    if( mode != currentSidelightsMode || color != currentSidelightsColor) {
        currentSidelightsMode = mode;
        currentSidelightsColor = color;
        return true;
    }
    return false;
}
