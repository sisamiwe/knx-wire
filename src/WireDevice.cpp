#include "WireDevice.h"
#include "knx.h"
#include "IncludeManager.h"
#include "KnxHelper.h"

WireDevice::WireDevice(/* args */)
{
}

WireDevice::~WireDevice()
{
}

void WireDevice::setValue(uint8_t iValue)
{
    if (mData.actor.lastOutputValue != iValue)
    {
        if (mDevice->setValue(iValue, mModelFunction))
            mData.actor.lastOutputValue = iValue;
    }
}

uint8_t WireDevice::getValue()
{
    uint8_t lResult = 0;
    if (mDevice != NULL) {
        mDevice->getValue(lResult, mModelFunction);
    }
    return lResult;
}

void WireDevice::clearSendDelay() {
    mData.sensor.sendDelay = 0;
}

bool WireDevice::isIO() {
    return (mDevice->Family() == MODEL_DS2413 || mDevice->Family() == MODEL_DS2408);
}

bool WireDevice::isIButton() {
    return (mDevice->Family() == MODEL_DS1990);
}

void WireDevice::processOneWire(uint8_t iDeviceIndex) {
    
    if (mDevice != NULL)
    {
        bool lLastSent = false;
        bool lNewState = false;
        uint8_t lValue = 0;
        switch (mDevice->Family())
        {
            case MODEL_DS18B20:
            case MODEL_DS18S20:
            case MODEL_DS2438:
                processSensor(10.0, iDeviceIndex * WIRE_ParamBlockSize + WIRE_ParamBlockOffset, iDeviceIndex + WIRE_KoOffset);
                break;
            case MODEL_DS1990:
                lLastSent = (mData.sensor.lastSentValue != 0);
                lNewState = (mDevice->Mode() == OneWire::Connected);
                if (lLastSent != lNewState)
                {
                    knx.getGroupObject(iDeviceIndex + WIRE_KoOffset).value(lNewState, getDPT(VAL_DPT_1));
                    printDebug("KO%d sendet Wert: %d\n", iDeviceIndex + WIRE_KoOffset, lNewState);
                    mData.sensor.lastSentValue = lNewState;
                }
                break;
            case MODEL_DS2408:
            case MODEL_DS2413:
                // handle I/O device, here just the input part (getting data from device)
                // Output is done in KO callback
                // now do input processing
                if (mDevice->Mode() == OneWire::Connected)
                {
                    lValue = getValue();
                    if (lValue != mData.actor.lastInputValue)
                    {
                        knx.getGroupObject(iDeviceIndex + WIRE_KoOffset).value(lValue, (mModelFunction == ModelFunction_IoByte) ? getDPT(VAL_DPT_5) : getDPT(VAL_DPT_1));
                        printDebug("KO%d sendet Wert: %0X\n", iDeviceIndex + WIRE_KoOffset, lValue);
                        mData.actor.lastInputValue = lValue;
                    }
                }
                break;
            default:
                break;
        }
    }
}

void WireDevice::setup(OneWire *iOneWire, uint8_t iModelFunction) {
    mDevice = iOneWire;
    mModelFunction = iModelFunction;
}

// generic sensor processing
void WireDevice::processSensor(float iOffsetFactor, uint16_t iParamIndex, uint16_t iKoNumber) {
    bool lForce = mData.sensor.sendDelay == 0;
    bool lSend = lForce;
    float lValueFactor = 1.0;
    // value factor depends on model funtion 
    if (mModelFunction >= ModelFunction_RawVDD && mModelFunction <= ModelFunction_RawVSens) {
        lValueFactor = 1000.0;
    }
    // process send cycle
    uint32_t lCycle = knx.paramInt(iParamIndex + WIRE_sSensorCycle) * 1000;

    // we waited enough, let's send the value
    if (lCycle && delayCheck(mData.sensor.sendDelay, lCycle))
        lSend = true;

    float lValue = 0;
    ;
    // process read cycle
    if (lSend || delayCheck(mData.sensor.readDelay, 5000))
    {
        // we waited enough, let's read the sensor
        int32_t lOffset = knx.paramByte(iParamIndex + WIRE_sSensorOffset);
        bool lValid = mDevice->getValue(lValue, mModelFunction);
        if (lValid)
        {
            // we have now the internal sensor value, we correct it now
            lValue = lValue * lValueFactor;
            lValue += (lOffset / iOffsetFactor);
            // smoothing (? glätten ?) of the new value
            // Formel: Value = ValueAlt + (ValueNeu - ValueAlt) / p
            if (!(lForce && mData.sensor.lastValue == 0.0))
            {
                lValue = mData.sensor.lastValue + (lValue - mData.sensor.lastValue) / knx.paramByte(iParamIndex + WIRE_sSensorSmooth);
            }
            // evaluate sending conditions (relative delta / absolute delta)
            if (mData.sensor.lastSentValue > 0.0)
            {
                // currently we asume indoor measurement with values > 0.0
                float lDelta = 100.0 - lValue / mData.sensor.lastSentValue * 100.0;
                uint32_t lPercent = knx.paramByte(iParamIndex + WIRE_sSensorDeltaPercent);
                if (lPercent && (uint32_t)abs(lDelta) >= lPercent)
                    lSend = true;
                // absolute diff we have to calculate in int due to rounding problems
                uint32_t lAbsolute = knx.paramWord(iParamIndex + WIRE_sSensorDeltaAbs);
                if (lAbsolute > 0 && abs(lValue * iOffsetFactor - mData.sensor.lastSentValue * iOffsetFactor) >= lAbsolute)
                    lSend = true;
            }
            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            mData.sensor.lastValue = lValue;
            knx.getGroupObject(iKoNumber).valueNoSend(lValue, getDPT(VAL_DPT_9));
            // wenn in KONNEKTING möglich, sollte der Wert im KO gespeichert werden, ohne dass
            // er gesendet wird, damit ein GroupValueRead zwischendurch auch den korrekten Wert liefert
            // hier lValue ins KO schreiben, KO-Nummer steht in iKoNumber
        }
        else
        {
            lSend = false;
        }
        mData.sensor.readDelay = millis();
    }
    if (lSend)
    {
        // wenn in KONNEKTING möglich, dann den letzten ins KO geschriebenen Wert jetzt senden lassen
        // sonst einfach lValue ueber das KO senden lassen, KO-Nummer steht in iKoNumber
        printDebug("KO%d sendet Wert: %f\n", iKoNumber, lValue);
        knx.getGroupObject(iKoNumber).objectWritten();
        mData.sensor.lastSentValue = (float)knx.getGroupObject(iKoNumber).value(getDPT(VAL_DPT_9));
        mData.sensor.sendDelay = millis();
        if (mData.sensor.sendDelay == 0)
            mData.sensor.sendDelay = 1;
    }
}
