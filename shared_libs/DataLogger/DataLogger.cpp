#include "DataLogger.h"

bool DataLogger::begin(const char* baseName, int csPin) {
    if (!SD.begin(csPin)) return (ok_ = false);

    int idx = 0; String fn;
    do { fn = String(baseName) + String(idx++) + ".csv"; } while (SD.exists(fn.c_str()));

    file_ = SD.open(fn.c_str(), FILE_WRITE);
    if (!file_) return (ok_ = false);

    file_.println(
        "t_ms,appsADC,pedalPct,bpsADC,brakePct,bppc,canStale,invFault,torqueCmd,"
        "erpm,duty,vin,iac,idc,ctrlTemp,motTemp,fault,id,iq,vLim,pLim,driveEn");
    file_.flush();
    Serial.println("SD logging -> " + fn);
    lastFlushMs_ = millis();
    return (ok_ = true);
}

void DataLogger::logRow(uint32_t t_ms,
                        uint16_t appsADC, float pedalPct,
                        uint16_t bpsADC,  float brakePct,
                        bool bppc, bool canStale, bool invFault,
                        float torqueCmd,
                        const DTI550_Data& d) {
    if (!ok_) return;

    file_.printf(
        "%lu,%u,%.1f,%u,%.1f,%d,%d,%d,%.1f,"
        "%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%.2f,%.2f,%d,%d,%u\n",
        (unsigned long)t_ms, appsADC, pedalPct, bpsADC, brakePct,
        bppc ? 1 : 0, canStale ? 1 : 0, invFault ? 1 : 0, torqueCmd,
        d.Actual_ERPM, d.Actual_Duty, d.Actual_InputVoltage,
        d.Actual_ACCurrent, d.Actual_DCCurrent, d.Actual_TempController, d.Actual_TempMotor,
        (unsigned)d.Actual_FaultCode, d.Actual_FOC_id, d.Actual_FOC_iq,
        d.Input_voltage_limit ? 1 : 0, d.Power_limit ? 1 : 0, (unsigned)d.Drive_enable);

    if ((uint32_t)(millis() - lastFlushMs_) >= FLUSH_MS) {
        file_.flush();                 // periodic sync, not per row
        lastFlushMs_ = millis();
    }
}
