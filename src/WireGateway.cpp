#include "Helper.h"
#include "Hardware.h"

#include "Sensor.h"
#include "OneWire.h"
#include "OneWireDS2482.h"
#include "WireBus.h"
#include "WireDevice.h"

// Reihenfolge beachten damit die Definitionen von Sensormodul.h ...
#include "WireGateway.h"
// ... auf jeden Fall Vorrang haben (beeinflussen auch die Logik)
// #include "../../knx-logic/src/LogikmodulCore.h"
#include "Logic.h"

const uint8_t cFirmwareMajor = 2;    // 0-31
const uint8_t cFirmwareMinor = 0;    // 0-31
const uint8_t cFirmwareRevision = 0; // 0-63

// Achtung: Bitfelder in der ETS haben eine gewöhnungswürdige
// Semantik: ein 1 Bit-Feld mit einem BitOffset=0 wird in Bit 7(!) geschrieben
uint32_t gStartupDelay;
uint32_t gHeartbeatDelay;
bool gIsRunning = false;
WireBus gBusMaster[3];
WireDevice gDevice[90];
uint16_t gCountSaveInterrupt = 0;
uint32_t gSaveInterruptTimestamp = 0;
bool gForceSensorRead = true;
Logic gLogic;

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
    if (gHeartbeatDelay == 0 || delayCheck(gHeartbeatDelay, knx.paramInt(LOG_Heartbeat) * 1000))
    {
        // we waited enough, let's send a heartbeat signal
        knx.getGroupObject(LOG_KoHeartbeat).value(true, getDPT(VAL_DPT_1));
        // if there is an error, we send it with heartbeat, too
        // if (knx.paramByte(LOG_Error) & 128) {
        //     if (getError()) sendError();
        // }
        gHeartbeatDelay = millis();
        // debug-helper for logic module
        // print("ParDewpoint: ");
        // println(knx.paramByte(LOG_Dewpoint));
        gLogic.debug();
    }
}

void ProcessReadRequests() {
    // this method is called after startup delay and executes read requests, which should just happen once after startup
    static bool sCalled = false;
    if (!sCalled) {
        gLogic.processReadRequests();
        sCalled = true;
    }
}

// true solgange der Start des gesamten Moduls verzögert werden soll
bool startupDelay()
{
    return !delayCheck(gStartupDelay, knx.paramInt(LOG_StartupDelay) * 1000);
}

void ProcessDiagnoseCommand(GroupObject &iKo) {
    // this method is called as soon as iKo is changed
    // an external change is expected
    // because this iKo is changed within this method,
    // the method is called again. This might result in
    // an endless loop. This is prevented by the isCalled pattern.
    static bool sIsCalled = false;
    if (!sIsCalled) {
        sIsCalled = true;
        uint8_t *lCommand = iKo.valueRef();
        bool lOutput = false;
        //diagnose is interactive and reacts on commands
        char lBuffer[16];
        if (lCommand[0] == 'v') {
            // Command v: retrun fimware version
            sprintf(lBuffer, "VER [%d] %d.%d", cFirmwareMajor, cFirmwareMinor, cFirmwareRevision);
            lOutput = true;
        } else if (lCommand[0] == 's') {
            // Command s: Number of save-Interupts (= false-save)
            sprintf(lBuffer, "SAVE %d", gCountSaveInterrupt);
            lOutput = true;
        } else {
            // let's check other modules for this command
            for (uint8_t i = 0; i < 14 && lCommand[i] > 0; i++)
                lBuffer[i] = lCommand[i];
            lOutput = gLogic.processDiagnoseCommand(lBuffer);
        }
        if (lOutput) {
            iKo.value(lBuffer, getDPT(VAL_DPT_16));
            printDebug("Diagnose: %s\n", lBuffer);
        }
        sIsCalled = false;
    }
};

void ProcessKoCallback(GroupObject &iKo)
{
    // check if we evaluate own KO
    if (iKo.asap() == LOG_KoDiagnose) {
        ProcessDiagnoseCommand(iKo);
    } else {
        // else dispatch to logicmodule
        gLogic.processInputKo(iKo);
    }
}

void appLoop()
{
    if (!knx.configured())
        return;

    // handle KNX stuff
    if (startupDelay())
        return;

    gIsRunning = true;

    // at this point startup-delay is done
    // we process heartbeat
    ProcessHeartbeat();
    ProcessReadRequests();
    gLogic.loop();
    gBusMaster[0].loop();
    gBusMaster[1].loop();
    gBusMaster[2].loop();

    // at Startup, we want to send all values immediately
    // ProcessSensors(gRuntimeData.forceSensorRead);
    gForceSensorRead = false;
}

void appSetup(bool iSaveSupported)
{
    // try to get rid of occasional I2C lock...
    // savePower();
    digitalWrite(PROG_LED_PIN, HIGH);
    digitalWrite(LED_YELLOW_PIN, HIGH);
    // delay(100);
    // restorePower();
    // check hardware availability
    boardCheck();
    digitalWrite(PROG_LED_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);

    if (knx.configured())
    {
        // 5 bit major, 5 bit minor, 6 bit revision
        knx.bau().deviceObject().version(cFirmwareMajor << 11 | cFirmwareMinor << 6 | cFirmwareRevision);
        gStartupDelay = millis();
        gHeartbeatDelay = 0;
        gCountSaveInterrupt = 0;
        // GroupObject &lKoRequestValues = knx.getGroupObject(LOG_KoRequestValues);
        if (GroupObject::classCallback() == 0) GroupObject::classCallback(ProcessKoCallback);
        gLogic.setup(iSaveSupported);
        gBusMaster[0].setup();
        gBusMaster[1].setup();
        gBusMaster[2].setup();
    }
}