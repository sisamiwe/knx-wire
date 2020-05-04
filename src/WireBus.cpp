// #include <knx.h>
#include <Arduino.h>
#include <Wire.h>
#include "Sensor.h"
#include "OneWire.h"
#include "OneWireDS2482.h"
#include "WireBus.h"

// OneWire
#define OFFSET_1W 26
#define OFFSET_1W_SENSOR 30
#define LEN_1W_SENSOR 20

#define KO_1W_NEWID 9
#define KO_1W_START 10

// ab hier sind es angenommene Parameterwerte, diese muessen dann
// aus passenden KONNEKTING-Parametern versorgt werden.
// Ich wuerde Dir empfehlen, die Reihenfolge der Parameter aber so zu belassen
// Der Einfachheit-halber habe ich die Parameter alle von einem Typ (int32_t) gemacht
// das kannst DU natuerlich bei KONNEKTING auch anders machen
int32_t konnParam[] = {
    // 1st 1W-Device (offset 30)
    0x28, 0xDC, 0xA5, 0x88, 0x0B, 0x00, 0x00, // ID Temp Einzelsensor Mat, KO 10
    OneWire::Temperature,                     // Modulfunktion
    // mit folgenden Settings für (Sensor | IO                    | iButton)
    0,  // Offset                         | IO-Bitmaske (1=Input) | Gruppe (0-keine, 1-32)
    11, // Sendeintervall (in s)
    0,  // senden bei absoluter Abweichung
    0,  // senden bei relativer Abweichung
    1,  // p fürs Glätten/Dämpfung
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 2nd 1W-Device (offset 50)
    0x28, 0x23, 0x3F, 0xDE, 0x03, 0x00, 0x00, // ID Temp Multisensor, KO 11
    OneWire::Temperature,                     // Modulfunktion
    // mit folgenden Settings für (Sensor | IO                    | iButton)
    0,  // Offset                         | IO-Bitmaske (1=Input) | Gruppe (0-keine, 1-32)
    13, // Sendeintervall (in s)
    0,  // senden bei absoluter Abweichung
    0,  // senden bei relativer Abweichung
    1,  // p fürs Glätten/Dämpfung
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 3rd 1W-Device (offset 70)
    0x28, 0x11, 0x88, 0x53, 0x07, 0x00, 0x00, // ID Temp Hum-Sensor, KO 12
    OneWire::Temperature,                     // Modulfunktion
    // mit folgenden Settings für (Sensor | IO                    | iButton)
    0,  // Offset                         | IO-Bitmaske (1=Input) | Gruppe (0-keine, 1-32)
    17, // Sendeintervall (in s)
    0,  // senden bei absoluter Abweichung
    0,  // senden bei relativer Abweichung
    1,  // p fürs Glätten/Dämpfung
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 4th 1W-Device (offset 90)
    0x01, 0x91, 0x6A, 0x59, 0x13, 0x00, 0x00, // ID iButton red, KO 13
    OneWire::Default,                         // Modulfunktion
    // mit folgenden Settings   (Sensor | IO                    | iButton)
    1, // Offset                        | IO-Bitmaske (1=Input) | Gruppe1 (0-keine, 1-32)
    5, //                               |                       | Gruppe 2
    7, //                               |                       | Gruppe 3
    0, //                               |                       | Gruppe 4
    0, //                               |                       | Gruppe 5
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 5th 1W-Device (offset 110)
    0x01, 0xB5, 0x07, 0x56, 0x16, 0x00, 0x00, // ID iButton green, KO 14
    OneWire::Default,                         // Modulfunktion
    // mit folgenden Settings   (Sensor | IO                    | iButton)
    1, // Offset                        | IO-Bitmaske (1=Input) | Gruppe1 (0-keine, 1-32)
    5, //                               |                       | Gruppe 2
    7, //                               |                       | Gruppe 3
    0, //                               |                       | Gruppe 4
    0, //                               |                       | Gruppe 5
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 6th 1W-Device (offset 130)
    0x3A, 0x3C, 0x82, 0x07, 0x00, 0x00, 0x00, // ID IO Multisensor, KO 15
    OneWire::IoByte,                          // Modulfunktion
    // mit folgenden Settings   (Sensor | IO                    | iButton)
    0, // Offset                        | IO-Bitmaske (1=Input) | Gruppe1 (0-keine, 1-32)
    3, //                               | Invertier-Bitmaske    | Gruppe 2
    0, //                               |                       | Gruppe 3
    0, //                               |                       | Gruppe 4
    0, //                               |                       | Gruppe 5
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    // 6th 1W-Device (offset 130)
    0x3A, 0x3C, 0x82, 0x07, 0x00, 0x00, 0x00, // ID IO Multisensor Bit 1, KO 16
    OneWire::IoBit1,                          // Modulfunktion
    // mit folgenden Settings   (Sensor | IO                    | iButton)
    0, // Offset                        | IO-Bitmaske (1=Input) | Gruppe1 (0-keine, 1-32)
    3, //                               | Invertier-Bitmaske    | Gruppe 2
    0, //                               |                       | Gruppe 3
    0, //                               |                       | Gruppe 4
    0, //                               |                       | Gruppe 5
    // fehlt noch Formel für generischen Analogeingang (DS2438)
    0, 0,
    0 // abschlußbyte, wahrscheinlich nicht nötig
};

// WireDevice::sDevice[COUNT_1WIRE_CHANNEL]; // list of all used devices accross all BM
uint8_t WireBus::sDeviceCount = 0;
uint8_t WireBus::sDeviceIndex = 0;
uint32_t WireBus::sUnknownDeviceDelay = 0;
WireDevice *WireBus::sDevice[COUNT_1WIRE_CHANNEL] = {0};

WireBus::WireBus()
    : gOneWireBM(0, WireBus::processNewIdCallback)
{
}

WireBus::~WireBus()
{
}

void WireBus::processKOCallback(uint16_t iKoIndex)
{
    // KO was written from KNX
    // in this simulation we assume it is a Output-KO
    uint8_t lDeviceIndex = iKoIndex - KO_1W_START;
    // For output we will write the value and validate the output (Status)
    // uint32_t lBitmaskIO = konnParam[OFFSET_1W_SENSOR + 8 + LEN_1W_SENSOR * lDeviceIndex]; // IO-Bitmask (1=Input, 0=Output)
    WireDevice *lDevice = sDevice[lDeviceIndex];
    lDevice->setValue(gOutput);
}

void WireBus::simulateKO()
{
    if (delayCheck(gOutputDelay, 1000))
    {
        gOutput++;
        processKOCallback(15);
        gOutputDelay = millis();
    }
}

// generic sensor processing
void WireBus::processSensor(sSensorInfo *cData, getSensorValue fGetSensorValue, MeasureType iMeasureType, float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber)
{
    bool lForce = cData->sendDelay == 0;
    bool lSend = lForce;

    // process send cycle
    uint32_t lCycle = konnParam[iParamIndex + 1] * 1000;

    // we waited enough, let's send the value
    if (lCycle && delayCheck(cData->sendDelay, lCycle))
        lSend = true;

    float lValue = 0;
    ;
    // process read cycle
    if (lSend || delayCheck(cData->readDelay, 5000))
    {
        // we waited enough, let's read the sensor
        int32_t lOffset = konnParam[iParamIndex];
        bool lValid = fGetSensorValue(iMeasureType, lValue);
        if (lValid)
        {
            // we have now the internal sensor value, we correct it now
            lValue += (lOffset / iOffsetFactor);
            lValue = lValue / iValueFactor;
            // smoothing (? glätten ?) of the new value
            // Formel: Value = ValueAlt + (ValueNeu - ValueAlt) / p
            if (!(lForce && cData->lastValue == 0.0))
            {
                lValue = cData->lastValue + (lValue - cData->lastValue) / konnParam[iParamIndex + 4];
            }
            // evaluate sending conditions (relative delta / absolute delta)
            if (cData->lastSentValue > 0.0)
            {
                // currently we asume indoor measurement with values > 0.0
                float lDelta = 100.0 - lValue / cData->lastSentValue * 100.0;
                uint32_t lPercent = konnParam[iParamIndex + 3];
                if (lPercent && (uint32_t)abs(lDelta) >= lPercent)
                    lSend = true;
                // absolute diff we have to calculate in int due to rounding problems
                uint32_t lAbsolute = konnParam[iParamIndex + 2];
                if (lAbsolute > 0 && abs(lValue * iOffsetFactor - cData->lastSentValue * iOffsetFactor) >= lAbsolute)
                    lSend = true;
            }
            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            cData->lastValue = lValue;
            // wenn in KONNEKTING möglich, sollte der Wert im KO gespeichert werden, ohne dass
            // er gesendet wird, damit ein GroupValueRead zwischendurch auch den korrekten Wert liefert
            // hier lValue ins KO schreiben, KO-Nummer steht in iKoNumber
        }
        else
        {
            lSend = false;
        }
        cData->readDelay = millis();
    }
    if (lSend)
    {
        // wenn in KONNEKTING möglich, dann den letzten ins KO geschriebenen Wert jetzt senden lassen
        // sonst einfach lValue ueber das KO senden lassen, KO-Nummer steht in iKoNumber
        printDebug("KO%d sendet Wert %f\n", iKoNumber, lValue);
        cData->lastSentValue = lValue;
        cData->sendDelay = millis();
        if (cData->sendDelay == 0)
            cData->sendDelay = 1;
    }
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
                printDebug("KO%d sendet Wert:", KO_1W_NEWID);
                printHEX(" ", lSensor->Id(), 7);
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

void WireBus::processOneWire(bool iForce)
{
    // are threre any OW sensors
    if (sDeviceCount > 0)
    {
        if (iForce)
        {
            for (uint8_t i = 0; i < sDeviceCount; i++)
            {
                sDevice[i]->clearSendDelay();
            }
        }
        // we iterate through all OW-Sensors
        sDevice[sDeviceIndex]->processOneWire();
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

    for (uint8_t lIndex = 0; lIndex < 30; lIndex++)
    {
        uint16_t lParamIndex = lIndex * LEN_1W_SENSOR + OFFSET_1W_SENSOR;
        if (konnParam[lParamIndex] > 0)
        {
            if (equalId(iOneWireSensor->Id(), konnParam + lParamIndex))
            {
                iOneWireSensor->setModeConnected(true);
                sDevice[lIndex]->setup(iOneWireSensor, static_cast<OneWire::ModelFunction>(konnParam[lParamIndex + 7]));
                sDeviceCount = max(sDeviceCount, lIndex + 1);
                lResult = true;
                break;
            }
        }
        else
        {
            break;
        }
    }
    // triger send new sensor info
    if (!lResult)
      sUnknownDeviceDelay = 0;
    return lResult;
}

void WireBus::setDeviceParameter(OneWire *iDevice, uint16_t iParamIndex)
{
    switch (iDevice->Family())
    {
        case MODEL_DS18B20:
        case MODEL_DS18S20:
            iDevice->setParameter(OneWire::MeasureResolution, 0x7F); //TEMP, will be set from ETS
            break;
        case MODEL_DS2408:
        case MODEL_DS2413:
            iDevice->setParameter(OneWire::IoMask, konnParam[iParamIndex + 8]);
            iDevice->setParameter(OneWire::IoInvertMask, konnParam[iParamIndex + 9]);
        default:
            break;
    }
}

void WireBus::publishSensors(OneWireDS2482 *iBM)
{
    OneWire *lSensor = NULL;
    tId lId;
    for (uint8_t lIndex = 0; lIndex < 30; lIndex++)
    {
        uint16_t lParamIndex = lIndex * LEN_1W_SENSOR + OFFSET_1W_SENSOR;
        if (konnParam[lParamIndex] > 0)
        {
            for (uint8_t i = 0; i < 7; i++)
                lId[i] = konnParam[lParamIndex + i];
            lSensor = iBM->CreateOneWire(lId);
            // lSensor->setModeConnected(true);
            lSensor->setModeDisconnected(true);
            setDeviceParameter(lSensor, lParamIndex);
            sDevice[lIndex]->setup(lSensor, static_cast<OneWire::ModelFunction>(konnParam[lParamIndex + 7]));
            sDeviceCount = max(sDeviceCount, lIndex + 1);
        }
        else
        {
            break;
        }
    }
}

void WireBus::loop()
{
    // hier muss noch der knx loop() hin
    // at Startup, we want to send all values immediately
    // ProcessSensors(gForceSensorRead);
    processOneWire(gForceSensorRead);

    // falls Du auch ein KO zum anfordern der Werte anbieten willst, muss in der Routine, die das KO auswertet
    // nur die folgende Variable auf true gesetzt werden, dann werden die Sensorwerte gesendet.
    gForceSensorRead = false;

    gOneWireBM.loop();
    simulateKO();
}

void WireBus::setup() {
    //currently empty
}
