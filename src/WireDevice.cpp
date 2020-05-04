#include "WireDevice.h"

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
}

void WireDevice::clearSendDelay() {
    mData.sensor.sendDelay = 0;
}

void WireDevice::processOneWire() {
    if (mDevice != NULL)
    {
        bool lLastSent = false;
        bool lNewState = false;
        uint8_t lValue = 0;
        switch (mDevice->Family())
        {
            case MODEL_DS18B20:
            case MODEL_DS18S20:
                // processSensor(&lOwData->sensor, measureOneWire, OneWireBM, 10.0, 1.0, OFFSET_1W_SENSOR + 8 + LEN_1W_SENSOR * lIndex, KO_1W_START + lIndex);
                break;
            case MODEL_DS1990:
                lLastSent = (mData.sensor.lastSentValue != 0);
                lNewState = (mDevice->Mode() == OneWire::Connected);
                if (lLastSent != lNewState)
                {
                    // printDebug("KO%d sendet Wert %d\n", KO_1W_START + lIndex, lNewState);
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
                        // printDebug("KO%d sendet Wert %0X\n", KO_1W_START + lIndex, lValue);
                        mData.actor.lastInputValue = lValue;
                    }
                }
                break;
            default:
                break;
        }
    }
}

void WireDevice::setup(OneWire *iOneWire, OneWire::ModelFunction iModelFunction)
{
    mDevice = iOneWire;
    mModelFunction = iModelFunction;
}
