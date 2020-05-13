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

WireBus::WireBus()
    : WireBus(0) {
}

WireBus::WireBus(uint8_t iI2cAddressOffset) : gOneWireBM(iI2cAddressOffset, WireBus::processNewIdCallback) {
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

// generic sensor processing
// void WireBus::processSensor(sSensorInfo *cData, getSensorValue fGetSensorValue, MeasureType iMeasureType, float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber)
// {
//     bool lForce = cData->sendDelay == 0;
//     bool lSend = lForce;

//     // process send cycle
//     uint32_t lCycle = konnParam[iParamIndex + 1] * 1000;

//     // we waited enough, let's send the value
//     if (lCycle && delayCheck(cData->sendDelay, lCycle))
//         lSend = true;

//     float lValue = 0;
//     ;
//     // process read cycle
//     if (lSend || delayCheck(cData->readDelay, 5000))
//     {
//         // we waited enough, let's read the sensor
//         int32_t lOffset = konnParam[iParamIndex];
//         bool lValid = fGetSensorValue(iMeasureType, lValue);
//         if (lValid)
//         {
//             // we have now the internal sensor value, we correct it now
//             lValue += (lOffset / iOffsetFactor);
//             lValue = lValue / iValueFactor;
//             // smoothing (? glätten ?) of the new value
//             // Formel: Value = ValueAlt + (ValueNeu - ValueAlt) / p
//             if (!(lForce && cData->lastValue == 0.0))
//             {
//                 lValue = cData->lastValue + (lValue - cData->lastValue) / konnParam[iParamIndex + 4];
//             }
//             // evaluate sending conditions (relative delta / absolute delta)
//             if (cData->lastSentValue > 0.0)
//             {
//                 // currently we asume indoor measurement with values > 0.0
//                 float lDelta = 100.0 - lValue / cData->lastSentValue * 100.0;
//                 uint32_t lPercent = konnParam[iParamIndex + 3];
//                 if (lPercent && (uint32_t)abs(lDelta) >= lPercent)
//                     lSend = true;
//                 // absolute diff we have to calculate in int due to rounding problems
//                 uint32_t lAbsolute = konnParam[iParamIndex + 2];
//                 if (lAbsolute > 0 && abs(lValue * iOffsetFactor - cData->lastSentValue * iOffsetFactor) >= lAbsolute)
//                     lSend = true;
//             }
//             // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
//             cData->lastValue = lValue;
//             // wenn in KONNEKTING möglich, sollte der Wert im KO gespeichert werden, ohne dass
//             // er gesendet wird, damit ein GroupValueRead zwischendurch auch den korrekten Wert liefert
//             // hier lValue ins KO schreiben, KO-Nummer steht in iKoNumber
//         }
//         else
//         {
//             lSend = false;
//         }
//         cData->readDelay = millis();
//     }
//     if (lSend)
//     {
//         // wenn in KONNEKTING möglich, dann den letzten ins KO geschriebenen Wert jetzt senden lassen
//         // sonst einfach lValue ueber das KO senden lassen, KO-Nummer steht in iKoNumber
//         printDebug("KO%d sendet Wert %f\n", iKoNumber, lValue);
//         cData->lastSentValue = lValue;
//         cData->sendDelay = millis();
//         if (cData->sendDelay == 0)
//             cData->sendDelay = 1;
//     }
// }

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
    processOneWire(gForceSensorRead);
    processUnknownDevices();
    // falls Du auch ein KO zum anfordern der Werte anbieten willst, muss in der Routine, die das KO auswertet
    // nur die folgende Variable auf true gesetzt werden, dann werden die Sensorwerte gesendet.
    gForceSensorRead = false;

    gOneWireBM.loop();
}

void WireBus::setup() {
    gOneWireBM.setup();
}
