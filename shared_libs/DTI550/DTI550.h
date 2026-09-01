#ifndef DTI550_H
#define DTI550_H

#include <stdint.h>

struct DTI550_Data
{

    // VECTOR__INDEPENDENT_SIG_MSG
    uint8_t NewSignal_0005;

    // HV500_ERPM_DUTY_VOLTAGE
    float Actual_ERPM;
    float Actual_Duty;
    float Actual_InputVoltage;

    // HV500_AC_DC_current
    float Actual_ACCurrent;
    float Actual_DCCurrent;

    // HV500_Temperatures
    float Actual_TempController;
    float Actual_TempMotor;
    uint8_t Actual_FaultCode;

    // HV500_FOC
    float Actual_FOC_id;
    float Actual_FOC_iq;

    // HV500_MISC
    float Actual_Throttle;
    float Actual_Brake;
    bool Digital_input_1;
    bool Digital_input_2;
    bool Digital_input_3;
    bool Digital_input_4;
    bool Digital_output_1;
    bool Digital_output_2;
    bool Digital_output_3;
    bool Digital_output_4;
    uint8_t Drive_enable;
    bool Capacitor_temp_limit;
    bool DC_current_limit;
    bool Drive_enable_limit;
    bool IGBT_accel_limit;
    bool IGBT_temp_limit;
    bool Input_voltage_limit;
    bool Motor_accel_limit;
    bool Motor_temp_limit;
    bool RPM_min_limit;
    bool RPM_max_limit;
    bool Power_limit;
    float CAN_map_version;

    // HV500_SetDriveEnable
    uint8_t CMD_DriveEnable;

    // HV500_SetAcCurrent
    float CMD_TargetAcCurrent;

    // HV500_SetBrakeCurrent
    float CMD_TargetBrakeCurrent;

    // HV500_SetERPM
    float CMD_TargetSpeed;

    // HV500_SetPosition
    float CMD_TargetPosition;

    // HV500_SetRelCurrent
    float CMD_TargetRelativeCurrent;

    // HV500_SetRelBrakeCurrent
    float CMD_TargeRelativeBrakeCurrent;

    // HV500_SetMaxAcCurrent
    float CMD_MaxAcCurrent;

    // HV500_SetMaxAcBrakeCurrent
    float CMD_MaxAcBrakeCurrent;

    // HV500_SetMaxDcCurrent
    float CMD_MaxDcCurrent;

    // HV500_SetMaxDcBrakeCurrent
    float CMD_MaxDcBrakeCurrent;

    // HV500_SetDigOutput
    bool CMD_SetDigOutput1;
    bool CMD_SetDigOutput2;
    bool CMD_SetDigOutput3;
    bool CMD_SetDigOutput4;

    // HV500_TargetIq
    uint8_t ControlMode;
    float TargetIq;
    float MotorPosition;
    uint8_t isMotorStill;

    // HV500_MinMaxAcCurrent
    float MaxAcCurrent;
    float AvailableMaxAcCurrent;
    float MinAcCurrent;
    float AvailableMinAcCurrent;

    // HV500_MinMaxDcCurrent
    float MaxDcCurrent;
    float AvailableMaxDcCurrent;
    float MinDcCurrent;
    float AvailableMinDcCurrent;
};

extern DTI550_Data dti550;

#endif
