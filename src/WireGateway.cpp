#include "Sensor.h"
#include "OneWire.h"
#include "OneWireDS2482.h"

#include "Hardware.h"

// Reihenfolge beachten damit die Definitionen von Sensormodul.h ...
#include "WireGateway.h"
// ... auf jeden Fall Vorrang haben (beeinflussen auch die Logik)
// #include "../../knx-logic/src/LogikmodulCore.h"
#include "Logic.h"

const uint8_t cFirmwareMajor = 2; //    0-31
const uint8_t cFirmwareMinor = 0; //    0-31
const uint8_t cFirmwareRevision = 0; // 0-63

// Achtung: Bitfelder in der ETS haben eine gewöhnungswürdige
// Semantik: ein 1 Bit-Feld mit einem BitOffset=0 wird in Bit 7(!) geschrieben

// Forward declarations
bool ProcessNewIdCallback(OneWire *iOneWireSensor);

// runtime information for the whole logik module
struct sSensorInfo
{
    float lastSentValue;
    unsigned long sendDelay;
    unsigned long readDelay;
};

struct sRuntimeInfo
{
    bool isRunning = false;
    sSensorInfo wire[8];
    unsigned long startupDelay;
    unsigned long heartbeatDelay;
    uint16_t countSaveInterrupt = 0;
    uint32_t saveInterruptTimestamp = 0;
    bool forceSensorRead = true;
};

sRuntimeInfo gRuntimeData;
uint8_t gSensor = 0;
Logic gLogic;
OneWireDS2482 gOneWireBM(ProcessNewIdCallback);

typedef bool (*getSensorValue)(MeasureType, float&);

// uint16_t getError() {
//     return (uint16_t)knx.getGroupObject(LOG_KoError).value(getDPT(VAL_DPT_7));
// }

// void setError(uint16_t iValue) {
//     knx.getGroupObject(LOG_KoError).valueNoSend(iValue, getDPT(VAL_DPT_7));
// }

// void sendError() {
//     knx.getGroupObject(LOG_KoError).objectWritten();
// }

void ProcessHeartbeat()
{
    // the first heartbeat is send directly after startup delay of the device
    if (gRuntimeData.heartbeatDelay == 0 || delayCheck(gRuntimeData.heartbeatDelay, knx.paramInt(LOG_Heartbeat) * 1000))
    {
        // we waited enough, let's send a heartbeat signal
        knx.getGroupObject(LOG_KoHeartbeat).value(true, getDPT(VAL_DPT_1));
        // if there is an error, we send it with heartbeat, too
        // if (knx.paramByte(LOG_Error) & 128) {
        //     if (getError()) sendError();
        // }
        gRuntimeData.heartbeatDelay = millis();
        // debug-helper for logic module
        // print("ParDewpoint: ");
        // println(knx.paramByte(LOG_Dewpoint));
        gLogic.debug();
    }
}

void ProcessReadRequests() {
    static bool sCalled = false;
    // the following code should be called only once
    if (!sCalled) {
        gLogic.processReadRequests();
        sCalled = true;
    }
}

// generic sensor processing
void ProcessSensor(sSensorInfo *cData, getSensorValue fGetSensorValue, MeasureType iMeasureType, float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber)
{   
    // we process just a sensor, which is selected in ETS
    if ((gSensor & iMeasureType) != iMeasureType) return;

    bool lForce = cData->sendDelay == 0;
    bool lSend = lForce;

    // process send cycle
    uint32_t lCycle = knx.paramInt(iParamIndex + 1) * 1000;

    // we waited enough, let's send the value
    if (lCycle && delayCheck(cData->sendDelay, lCycle))
        lSend = true;

    // process read cycle
    if (lForce || delayCheck(cData->readDelay, 5000))
    {
        // we waited enough, let's read the sensor
        int32_t lOffset = (int8_t)knx.paramByte(iParamIndex);
        float lValue;
        bool lValid = fGetSensorValue(iMeasureType, lValue);
        if (lValid) {
            // we have now the internal sensor value, we correct it now
            lValue += (lOffset / iOffsetFactor);
            lValue = lValue / iValueFactor;
            // if there are external values to take into account, we do it here
            uint8_t lNumExternalValues = knx.paramByte(iParamIndex + 9) & 3;
            float lDivisor = 1.0f;
            float lDivident = 0.0f;
            float lFactor = 0.0f;
            if (lDivisor > 0.1f) {
                // smoothing (? glätten ?) of the new value
                // Formel: Value = ValueAlt + (ValueNeu - ValueAlt) / p
                float lValueAlt = (float)knx.getGroupObject(iKoNumber).value(getDPT(VAL_DPT_9));
                if (!(lForce && lValueAlt == 0.0f)) {
                    lValue = lValueAlt + (lValue - lValueAlt) / knx.paramByte(iParamIndex + 8);
                }
                // evaluate sending conditions (relative delta / absolute delta)
                if (cData->lastSentValue > 0.0f) {
                    // currently we asume indoor measurement with values > 0.0
                    float lDelta = 100.0f - lValue / cData->lastSentValue * 100.0f;
                    uint8_t lPercent = knx.paramByte(iParamIndex + 7);
                    if (lPercent > 0 && (uint8_t)round(abs(lDelta)) >= lPercent)
                        lSend = true;
                    float lAbsolute = knx.paramWord(iParamIndex + 5) / iOffsetFactor;
                    if (lAbsolute > 0.0f && round(abs(lValue - cData->lastSentValue)) >= lAbsolute)
                        lSend = true;
                }
                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                knx.getGroupObject(iKoNumber).valueNoSend(lValue, getDPT(VAL_DPT_9));
            }
        } else {
            lSend = false;
        }
        cData->readDelay = millis();
    }
    if (lSend)
    {
        // if ((getError() & iMeasureType) == 0) {
            knx.getGroupObject(iKoNumber).objectWritten();
            cData->lastSentValue = (float)knx.getGroupObject(iKoNumber).value(getDPT(VAL_DPT_9));
        // }
        cData->sendDelay = millis();
        if (cData->sendDelay == 0) cData->sendDelay = 1;
    }
}

// true solgange der Start des gesamten Moduls verzögert werden soll
bool startupDelay()
{
    return !delayCheck(gRuntimeData.startupDelay, knx.paramInt(LOG_StartupDelay) * 1000);
}

bool ProcessNewIdCallback(OneWire *iOneWireSensor) {
    // temporär
    uint8_t lLocalId[7] = {0x28, 0xDC, 0xA5, 0x88, 0x0B, 0x00, 0x00};

    if (equalId(iOneWireSensor->Id(), lLocalId)) {
        iOneWireSensor->setModeConnected();
    }
    return true;
}

void ProcessDiagnoseCommand(GroupObject &iKo) {
    uint8_t *lCommand = iKo.valueRef();
    //diagnose is interactive and reacts on commands
    char lBuffer[16];
    if (lCommand[0] == 'v') {
        // Command v: retrun fimware version
        sprintf(lBuffer, "VER [%d] %d.%d", cFirmwareMajor, cFirmwareMinor, cFirmwareRevision);
        iKo.value(lBuffer, getDPT(VAL_DPT_16));
    } else if (lCommand[0] == 's') {
        // Command s: Number of save-Interupts (= false-save)
        sprintf(lBuffer, "SAVE %d", gRuntimeData.countSaveInterrupt);
        iKo.value(lBuffer, getDPT(VAL_DPT_16));
    }
};

void ProcessKoCallback(GroupObject &iKo)
{
    // check if we evaluate own KO
    // if (iKo.asap() == LOG_KoDiagnose) {
    //     ProcessDiagnoseCommand(iKo);
    // } else {
        // else dispatch to logicmodule
        gLogic.processInputKo(iKo);
    // }
}

void ProcessInterrupt() {
    if (gRuntimeData.saveInterruptTimestamp) {
        printDebug("Sensormodul: SAVE-Interrupt processing started after %lu ms\n", millis() - gRuntimeData.saveInterruptTimestamp);
        gRuntimeData.saveInterruptTimestamp = millis();
        // for the moment, we send only an Info on error object in case of an save interrumpt
        // uint16_t lError = getError();
        // setError(lError | 128);
        // sendError();
        // switch off all energy intensive hardware to gain time for EEPROM write
        savePower();
        // call according logic interrupt handler
        gLogic.processInterrupt(true);
        Sensor::saveState();
        printDebug("Sensormodul: SAVE-Interrupt processing duration was %lu ms\n", millis() - gRuntimeData.saveInterruptTimestamp);
        // in case, SaveInterrupt was a false positive
        // we restore power and I2C-Bus
        Wire.end();
        // wait another 200 ms
        delay(200);
        restorePower();
        delay(100);
        Wire.begin();
        // Sensor::restartSensors();
        gRuntimeData.saveInterruptTimestamp = 0;
    }
}


void appLoop()
{

    if (!knx.configured())
        return;

    // handle KNX stuff
    if (startupDelay())
        return;

    gRuntimeData.isRunning = true;
    ProcessInterrupt();

    // at this point startup-delay is done
    // we process heartbeat
    ProcessHeartbeat();
    ProcessReadRequests();
    gLogic.loop();
    gOneWireBM.loop();
    
    // at Startup, we want to send all values immediately
    // ProcessSensors(gRuntimeData.forceSensorRead);
    gRuntimeData.forceSensorRead = false;

    Sensor::sensorLoop();
}

// handle interrupt from save pin
void onSafePinInterruptHandler() {
    gRuntimeData.countSaveInterrupt += 1;
    gRuntimeData.saveInterruptTimestamp = millis();
    // gLogic.onSafePinInterruptHandler();
}

void beforeRestartHandler() {
    printDebug("before Restart called\n");
    Sensor::saveState();
    gLogic.onBeforeRestartHandler();
    // we try get a clean state on I2C bus
    Wire.end();
}

void beforeTableUnloadHandler(TableObject& iTableObject, LoadState& iNewState) {
    static uint32_t sLastCalled = 0;
    printDebug("Table changed called with state %d\n", iNewState);
    
    if (iNewState == 0) {
        printDebug("Table unload called\n");
        if (sLastCalled == 0 || delayCheck(sLastCalled, 10000)) {
            Sensor::saveState();
            gLogic.onBeforeTableUnloadHandler(iTableObject, iNewState);
            sLastCalled = millis();
        }
    }
}

void appSetup(bool iSaveSupported)
{
    // try to get rid of occasional I2C lock...
    savePower();
    delay(100);
    restorePower();
    // check hardware availability
    boardCheck();

    if (knx.configured())
    {
        // 5 bit major, 5 bit minor, 6 bit revision
        knx.bau().deviceObject().version(cFirmwareMajor << 11 | cFirmwareMinor << 6 | cFirmwareRevision);
        // gSensor = (knx.paramByte(LOG_SensorDevice));
        gRuntimeData.startupDelay = millis();
        gRuntimeData.heartbeatDelay = 0;
        gRuntimeData.countSaveInterrupt = 0;
        // GroupObject &lKoRequestValues = knx.getGroupObject(LOG_KoRequestValues);
        if (GroupObject::classCallback() == 0) GroupObject::classCallback(ProcessKoCallback);
        if (knx.getBeforeRestartCallback() == 0) knx.addBeforeRestartCallback(beforeRestartHandler);
        if (TableObject::getBeforeTableUnloadCallback() == 0) TableObject::addBeforeTableUnloadCallback(beforeTableUnloadHandler);
        // StartSensor();
#ifdef SAVE_INTERRUPT_PIN
        if (iSaveSupported)
            attachInterrupt(digitalPinToInterrupt(SAVE_INTERRUPT_PIN), onSafePinInterruptHandler, FALLING);
#endif
        gLogic.setup(false);
        // if (knx.paramByte(LOG_SensorDevice) & BIT_1WIRE)
        gOneWireBM.setup();
    }
}