// #include <knx.h>
#include <Arduino.h>
#include <Wire.h>
#include <knx.h>
#include "Sensor.h"
#include "OneWire.h"
#include "OneWireDS2482.h"
#include "WireBus.h"
#include "KnxHelper.h"

#include "IncludeManager.h"

// WireDevice::sDevice[COUNT_1WIRE_CHANNEL]; // list of all used devices accross all BM
uint8_t WireBus::sDeviceCount = 0;
uint8_t WireBus::sDeviceIndex = 0;
uint32_t WireBus::sUnknownDeviceDelay = 0;
WireDevice *WireBus::sDevice[COUNT_1WIRE_CHANNEL] = {0};

WireBus::WireBus() : gOneWireBM(WireBus::processNewIdCallback, WireBus::knxLoopCallback) {
}

WireBus::~WireBus() {
}

void WireBus::processKOCallback(GroupObject &iKo)
{
    // check for 1-Wire-KO
    if (iKo.asap() >= WIRE_KoOffset && iKo.asap() < ((knx.paramByte(LOG_BusMasterCount) & 0x60) >> 5) * 30 + WIRE_KoOffset) {
        uint8_t lDeviceIndex = iKo.asap() - WIRE_KoOffset;
        // has to be an input KO (for an 1W output device)
        uint16_t lParamIndex = lDeviceIndex * WIRE_ParamBlockSize + WIRE_ParamBlockOffset;
        WireDevice *lDevice = sDevice[lDeviceIndex];
        // we have to check this (in case someone writes on a KO of a sensor device)
        if (lDevice->isIO()) {
            // find correct DPT for KO
            if (knx.paramByte(lParamIndex + WIRE_sModelFunction) == ModelFunction_IoByte)
                lDevice->setValue(iKo.value(getDPT(VAL_DPT_5)));
            else
                lDevice->setValue(iKo.value(getDPT(VAL_DPT_1)));
        }
    }
}

// callback to a method of an instance (Pattern)
// this static callback gets a pointer to the instance
void WireBus::loopCallback(void *iThis)
{
    WireBus *self = static_cast<WireBus *>(iThis);
    self->loop();
}

void WireBus::knxLoopCallback() {
    knx.loop();
}

void WireBus::processUnknownDevices()
{
    static uint8_t sIterator = 0;
    static uint8_t sDelayFactor = 2;
    bool lForce = sUnknownDeviceDelay == 0;

    if (gOneWireBM.DeviceCount() > 0 && (lForce || delayCheck(sUnknownDeviceDelay, sDelayFactor * 1000)))
    {
        for (; sIterator < gOneWireBM.DeviceCount();)
        {
            OneWire *lSensor = gOneWireBM.Sensor(sIterator++);
            if (lSensor->Mode() == OneWire::New)
            {
                // output is 1 new ID in 2 Seconds at max
                printDebug("KO%d sendet Wert: ", LOG_KoNewId);
                char lBuffer[15];
                lBuffer[14] = 0;
                sprintf(lBuffer, "%02X%02X%02X%02X%02X%02X%02X", lSensor->Id()[0], lSensor->Id()[1], lSensor->Id()[2], lSensor->Id()[3], lSensor->Id()[4], lSensor->Id()[5], lSensor->Id()[6]);
                printDebug("%s\n", lBuffer);
                knx.getGroupObject(LOG_KoNewId).value(lBuffer, getDPT(VAL_DPT_16));
                sDelayFactor = 2; // check in 2 Seconds for next new ID
                break;
            }
        }
        if (sIterator == gOneWireBM.DeviceCount())
        {
            sIterator = 0;
            sDelayFactor = 60; // next output of all IDs in a minute
        }
        sUnknownDeviceDelay = millis();
        if (sUnknownDeviceDelay == 0)
            sUnknownDeviceDelay = 1;
    }
}

// static - this is not perfect, but it works
bool WireBus::measureOneWire(MeasureType iMeasureType, float &eValue)
{
    eValue = sDevice[sDeviceIndex]->getValue();
    return true;
}

// static
void WireBus::processIButtonGroups()
{
    // group processing would be a loop over max 90 devices 
    // times 8 Groups and the according logical operations
    // this would block too long!
    // Here we implement an asynchroneous algorithm
    // as described in doc/iButton-Group-Handling.txt
    // so we iterate per device just through all 8 groups
    // final group result takes at max 90 iterations
    static uint8_t sIndex = 0;
    static bool sIButtonExist = true;
    static uint8_t sToProcess = 0xFF;
    static uint8_t sGroupType = knx.paramByte(LOG_Group1); // this is constant for the whole program runtime
    WireDevice *lDevice;

    // we iterate only if there are iButtons
    // if (sIButtonExist) {
        sIButtonExist = false;
        // create start condition
        if (sIndex == 0) {
            sToProcess = 0xFF;
        }
        // search for the next iButton
        do {
            lDevice = sDevice[sIndex++];
        } while (sIndex < sDeviceCount && !lDevice->isIButton());
        // process this iButton
        if (sIndex <= sDeviceCount && lDevice->isIButton()) {
            sIButtonExist = true;
            bool lValue = lDevice->getValue();
            uint8_t lButtonGroups = knx.paramByte((sIndex - 1) * WIRE_ParamBlockSize + WIRE_ParamBlockOffset + WIRE_sGroup1);
            uint8_t lButtonGroupsToProcess = lButtonGroups & sToProcess;
            uint8_t lGroupBit = 0x80;
            for (uint8_t lGroupIndex = 0; lGroupIndex < 7 && lButtonGroupsToProcess; lGroupIndex++)
            {
                if (lButtonGroupsToProcess & lGroupBit) {
                    //group has to be processed
                    bool lGroupType = sGroupType & lGroupBit;
                    if (lGroupType != lValue) {
                        // this is the case, where 
                        // - group type is AND and value is 0 or
                        // - group type is OR and value is 1
                        // we set the group KO to value...
                        GroupObject &lKo = knx.getGroupObject(LOG_KoGroup1 + lGroupIndex);
                        if ((bool)lKo.value(getDPT(VAL_DPT_1)) != lValue) {
                            printDebug("KO%d sendet Wert: %d\n", LOG_KoGroup1 + lGroupIndex, lValue);
                            lKo.value(lValue, getDPT(VAL_DPT_1));
                        }
                        // and mark this group as processed
                        sToProcess &= ~lGroupBit;
                    }
                }
                lGroupBit >>= 1;
                knx.loop();
            }
        }
        if (sIndex >= sDeviceCount && sIButtonExist) {
            // we iterated through all iButtons, let's process remaining groups
            uint8_t lGroupBit = 0x80;
            for (uint8_t lGroupIndex = 0; lGroupIndex < 7; lGroupIndex++)
            {
                if (sToProcess & lGroupBit) {
                    // group was not processed, we set KO to group type
                    GroupObject &lKo = knx.getGroupObject(LOG_KoGroup1 + lGroupIndex);
                    bool lValue = (sGroupType & lGroupBit);
                    if ((bool)lKo.value(getDPT(VAL_DPT_1)) != lValue)
                    {
                        printDebug("KO%d sendet Wert: %d\n", LOG_KoGroup1 + lGroupIndex, lValue);
                        lKo.value(lValue, getDPT(VAL_DPT_1));
                    }
                }
                lGroupBit >>= 1;
                knx.loop();
            }
        }
        if (sIndex >= sDeviceCount)
            sIndex = 0;
    // }
}

void WireBus::processOneWire(bool iForce) {
    // are threre any OW sensors
    if (sDeviceCount > 0)
    {
        if (iForce) {
            for (uint8_t i = 0; i < sDeviceCount; i++)
                sDevice[i]->clearSendDelay();
        }
        // we iterate through all OW-Sensors
        sDevice[sDeviceIndex]->processOneWire(sDeviceIndex);
        if (++sDeviceIndex >= sDeviceCount)
            sDeviceIndex = 0;
    }
}

// jeder neue Sensor, der erstmals bei der 1W-Suche erkannt wird,
// wird über diesen Callback der Applikation mitgeteilt.
// Hier wird geprüft, ob dieser Sensor parametriert worden ist.
// Wenn ja, wird sein Status auf Connected gesetzt und
// der Sensor mit dem Parametersatz gespeichert.
// static
bool WireBus::processNewIdCallback(OneWire *iOneWireSensor)
{
    bool lResult = false;
    uint8_t lBusMasterCount = ((knx.paramByte(LOG_BusMasterCount) & 0x60) >> 5) * 30;
    for (uint8_t lIndex = 0; lIndex < lBusMasterCount; lIndex++)
    {
        uint16_t lParamIndex = lIndex * WIRE_ParamBlockSize + WIRE_ParamBlockOffset;
        if (knx.paramByte(lParamIndex) > 0) {
            if (equalId(iOneWireSensor->Id(), knx.paramData(lParamIndex)))
            {
                iOneWireSensor->setModeConnected(true);
                sDevice[lIndex] = new WireDevice();
                sDevice[lIndex]->setup(iOneWireSensor, knx.paramByte(lParamIndex + WIRE_sModelFunction));
                setDeviceParameter(iOneWireSensor, lParamIndex);
                sDeviceCount = max(sDeviceCount, lIndex + 1);
                lResult = true;
            }
        }
    }
    // triger send new sensor info
    if (!lResult)
        sUnknownDeviceDelay = millis() - 58000; // start output in 2 seconds
    return lResult;
}

void WireBus::setDeviceParameter(OneWire *iDevice, uint16_t iParamIndex)
{
    uint8_t lModelFunction = knx.paramByte(iParamIndex + WIRE_sModelFunction);
    switch (iDevice->Family())
    {
        case MODEL_DS18B20:
        case MODEL_DS18S20:
            iDevice->setParameter(OneWire::MeasureResolution, 0x7F, lModelFunction); //TEMP, will be set from ETS
            break;
        case MODEL_DS2408:
        case MODEL_DS2413:
            iDevice->setParameter(OneWire::IoMask, knx.paramByte(iParamIndex + WIRE_sIoBitmask0), lModelFunction);
            iDevice->setParameter(OneWire::IoInvertMask, knx.paramByte(iParamIndex + WIRE_sIoInvertBitmask0), lModelFunction);
        default:
            break;
    }
}

// void WireBus::publishSensors(OneWireDS2482 *iBM)
// {
//     OneWire *lSensor = NULL;
//     tId lId;
//     for (uint8_t lIndex = 0; lIndex < 30; lIndex++)
//     {
//         uint16_t lParamIndex = lIndex * LEN_1W_SENSOR + OFFSET_1W_SENSOR;
//         if (konnParam[lParamIndex] > 0)
//         {
//             for (uint8_t i = 0; i < 7; i++)
//                 lId[i] = konnParam[lParamIndex + i];
//             lSensor = iBM->CreateOneWire(lId);
//             // lSensor->setModeConnected(true);
//             lSensor->setModeDisconnected(true);
//             setDeviceParameter(lSensor, lParamIndex);
//             sDevice[lIndex]->setup(lSensor, static_cast<OneWire::ModelFunction>(konnParam[lParamIndex + 7]));
//             sDeviceCount = max(sDeviceCount, lIndex + 1);
//         }
//         else
//         {
//             break;
//         }
//     }
// }

void WireBus::loop()
{
    knx.loop(); // improve knx responsiveness

    if (!gIsSetup)
        return;

    processOneWire(gForceSensorRead);
    processUnknownDevices();
    processIButtonGroups();
    // falls Du auch ein KO zum anfordern der Werte anbieten willst, muss in der Routine, die das KO auswertet
    // nur die folgende Variable auf true gesetzt werden, dann werden die Sensorwerte gesendet.
    gForceSensorRead = false;

    gOneWireBM.loop();
}

void WireBus::setup(uint8_t iI2cAddressOffset, bool iSearchNewDevices, bool iSearchIButtons)
{
    uint8_t lBusMasterMemOffset = 2 * iI2cAddressOffset;
    gOneWireBM.setup(iI2cAddressOffset, iSearchNewDevices, iSearchIButtons,
                     (knx.paramByte(LOG_Busmaster1RSTL + lBusMasterMemOffset) & LOG_Busmaster1RSTLMask) >> LOG_Busmaster1RSTLShift,
                     (knx.paramByte(LOG_Busmaster1MSP  + lBusMasterMemOffset) & LOG_Busmaster1MSPMask)  >> LOG_Busmaster1MSPShift,
                     (knx.paramByte(LOG_Busmaster1W0L  + lBusMasterMemOffset) & LOG_Busmaster1W0LMask)  >> LOG_Busmaster1W0LShift,
                     (knx.paramByte(LOG_Busmaster1REC0 + lBusMasterMemOffset) & LOG_Busmaster1REC0Mask) >> LOG_Busmaster1REC0Shift,
                     (knx.paramByte(LOG_Busmaster1WPU  + lBusMasterMemOffset) & LOG_Busmaster1WPUMask)  >> LOG_Busmaster1WPUShift);
    gIsSetup = true;
}
