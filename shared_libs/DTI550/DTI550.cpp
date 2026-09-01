#include "DTI550.h"
#include "bitextract.h"
#include <FlexCAN_T4.h>

DTI550_Data dti550;

void decode_3221225472(const CAN_message_t& msg)
{
    dti550.NewSignal_0005 = extractUnsigned(msg.buf, 39, 8, false);
}

void decode_1028(const CAN_message_t& msg)
{
    dti550.Actual_ERPM = extractSigned(msg.buf, 7, 32, false);
    dti550.Actual_Duty = extractSigned(msg.buf, 39, 16, false) * 0.1 + 0.0;
    dti550.Actual_InputVoltage = extractSigned(msg.buf, 55, 16, false);
}

void decode_1060(const CAN_message_t& msg)
{
    dti550.Actual_ACCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
    dti550.Actual_DCCurrent = extractSigned(msg.buf, 23, 16, false) * 0.1 + 0.0;
}

void decode_1092(const CAN_message_t& msg)
{
    dti550.Actual_TempController = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
    dti550.Actual_TempMotor = extractSigned(msg.buf, 23, 16, false) * 0.1 + 0.0;
    dti550.Actual_FaultCode = extractUnsigned(msg.buf, 39, 8, false);
}

void decode_1124(const CAN_message_t& msg)
{
    dti550.Actual_FOC_id = extractSigned(msg.buf, 7, 32, false) * 0.01 + 0.0;
    dti550.Actual_FOC_iq = extractSigned(msg.buf, 39, 32, false) * 0.01 + 0.0;
}

void decode_1156(const CAN_message_t& msg)
{
    dti550.Actual_Throttle = extractSigned(msg.buf, 7, 8, false);
    dti550.Actual_Brake = extractSigned(msg.buf, 15, 8, false);
    dti550.Digital_input_1 = extractUnsigned(msg.buf, 16, 1, false);
    dti550.Digital_input_2 = extractUnsigned(msg.buf, 17, 1, false);
    dti550.Digital_input_3 = extractUnsigned(msg.buf, 18, 1, false);
    dti550.Digital_input_4 = extractUnsigned(msg.buf, 19, 1, false);
    dti550.Digital_output_1 = extractUnsigned(msg.buf, 20, 1, false);
    dti550.Digital_output_2 = extractUnsigned(msg.buf, 21, 1, false);
    dti550.Digital_output_3 = extractUnsigned(msg.buf, 22, 1, false);
    dti550.Digital_output_4 = extractUnsigned(msg.buf, 23, 1, false);
    dti550.Drive_enable = extractUnsigned(msg.buf, 31, 8, false);
    dti550.Capacitor_temp_limit = extractUnsigned(msg.buf, 32, 1, false);
    dti550.DC_current_limit = extractUnsigned(msg.buf, 33, 1, false);
    dti550.Drive_enable_limit = extractUnsigned(msg.buf, 34, 1, false);
    dti550.IGBT_accel_limit = extractUnsigned(msg.buf, 35, 1, false);
    dti550.IGBT_temp_limit = extractUnsigned(msg.buf, 36, 1, false);
    dti550.Input_voltage_limit = extractUnsigned(msg.buf, 37, 1, false);
    dti550.Motor_accel_limit = extractUnsigned(msg.buf, 38, 1, false);
    dti550.Motor_temp_limit = extractUnsigned(msg.buf, 39, 1, false);
    dti550.RPM_min_limit = extractUnsigned(msg.buf, 40, 1, false);
    dti550.RPM_max_limit = extractUnsigned(msg.buf, 41, 1, false);
    dti550.Power_limit = extractUnsigned(msg.buf, 42, 1, false);
    dti550.CAN_map_version = extractUnsigned(msg.buf, 63, 8, false) * 0.1 + 0.0;
}

void decode_388(const CAN_message_t& msg)
{
    dti550.CMD_DriveEnable = extractUnsigned(msg.buf, 7, 8, false);
}

void decode_36(const CAN_message_t& msg)
{
    dti550.CMD_TargetAcCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_68(const CAN_message_t& msg)
{
    dti550.CMD_TargetBrakeCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_100(const CAN_message_t& msg)
{
    dti550.CMD_TargetSpeed = extractSigned(msg.buf, 7, 32, false);
}

void decode_132(const CAN_message_t& msg)
{
    dti550.CMD_TargetPosition = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_164(const CAN_message_t& msg)
{
    dti550.CMD_TargetRelativeCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_196(const CAN_message_t& msg)
{
    dti550.CMD_TargeRelativeBrakeCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_260(const CAN_message_t& msg)
{
    dti550.CMD_MaxAcCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_292(const CAN_message_t& msg)
{
    dti550.CMD_MaxAcBrakeCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_324(const CAN_message_t& msg)
{
    dti550.CMD_MaxDcCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_356(const CAN_message_t& msg)
{
    dti550.CMD_MaxDcBrakeCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
}

void decode_228(const CAN_message_t& msg)
{
    dti550.CMD_SetDigOutput1 = extractUnsigned(msg.buf, 0, 1, false);
    dti550.CMD_SetDigOutput2 = extractUnsigned(msg.buf, 1, 1, false);
    dti550.CMD_SetDigOutput3 = extractUnsigned(msg.buf, 2, 1, false);
    dti550.CMD_SetDigOutput4 = extractUnsigned(msg.buf, 3, 1, false);
}

void decode_996(const CAN_message_t& msg)
{
    dti550.ControlMode = extractUnsigned(msg.buf, 7, 8, false);
    dti550.TargetIq = extractSigned(msg.buf, 15, 16, false) * 0.1 + 0.0;
    dti550.MotorPosition = extractUnsigned(msg.buf, 31, 16, false) * 0.1 + 0.0;
    dti550.isMotorStill = extractUnsigned(msg.buf, 47, 8, false);
}

void decode_1188(const CAN_message_t& msg)
{
    dti550.MaxAcCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
    dti550.AvailableMaxAcCurrent = extractSigned(msg.buf, 23, 16, false) * 0.1 + 0.0;
    dti550.MinAcCurrent = extractSigned(msg.buf, 39, 16, false) * 0.1 + 0.0;
    dti550.AvailableMinAcCurrent = extractSigned(msg.buf, 55, 16, false) * 0.1 + 0.0;
}

void decode_1220(const CAN_message_t& msg)
{
    dti550.MaxDcCurrent = extractSigned(msg.buf, 7, 16, false) * 0.1 + 0.0;
    dti550.AvailableMaxDcCurrent = extractSigned(msg.buf, 23, 16, false) * 0.1 + 0.0;
    dti550.MinDcCurrent = extractSigned(msg.buf, 39, 16, false) * 0.1 + 0.0;
    dti550.AvailableMinDcCurrent = extractSigned(msg.buf, 55, 16, false) * 0.1 + 0.0;
}

void DTI550_decode(const CAN_message_t& msg)
{
    switch(msg.id)
    {
        case 3221225472: decode_3221225472(msg); break;
        case 1028: decode_1028(msg); break;
        case 1060: decode_1060(msg); break;
        case 1092: decode_1092(msg); break;
        case 1124: decode_1124(msg); break;
        case 1156: decode_1156(msg); break;
        case 388: decode_388(msg); break;
        case 36: decode_36(msg); break;
        case 68: decode_68(msg); break;
        case 100: decode_100(msg); break;
        case 132: decode_132(msg); break;
        case 164: decode_164(msg); break;
        case 196: decode_196(msg); break;
        case 260: decode_260(msg); break;
        case 292: decode_292(msg); break;
        case 324: decode_324(msg); break;
        case 356: decode_356(msg); break;
        case 228: decode_228(msg); break;
        case 996: decode_996(msg); break;
        case 1188: decode_1188(msg); break;
        case 1220: decode_1220(msg); break;
        default:
            break;
    }
}
