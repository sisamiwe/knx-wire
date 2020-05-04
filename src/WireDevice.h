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
    OneWire::ModelFunction mModelFunction;
    OneWire *mDevice = NULL;
    uData mData;

  public:
    WireDevice();
    ~WireDevice();

    void setValue(uint8_t iValue);
    uint8_t getValue();
    void clearSendDelay();
    void processOneWire();

    void setup(OneWire *iOneWire, OneWire::ModelFunction iModelFunction);
};