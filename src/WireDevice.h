#pragma once
#include <OneWire.h>

struct sSensorInfo
{
    float lastValue;
    float lastSentValue;
    uint32_t sendDelay;
    uint32_t readDelay;
};

struct sActorInfo
{
    uint8_t lastOutputValue;
    uint8_t lastInputValue;
    uint32_t sendDelay;
    uint32_t readDelay;
};

union uData
{
    sSensorInfo sensor;
    sActorInfo actor;
};

class WireDevice
{
  private:
    uint8_t mModelFunction;
    OneWire *mDevice = NULL;
    uData mData;

  public:
    WireDevice();
    ~WireDevice();

    void setValue(uint8_t iValue);
    uint8_t getValue();
    void clearSendDelay();
    void processOneWire(uint8_t iDeviceIndex);
    bool isIO();
    bool isIButton();

    void setup(OneWire *iOneWire, uint8_t iModelFunction);
    void processSensor(float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber);
};
