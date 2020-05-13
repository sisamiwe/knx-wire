#include "knx.h"
#include "Hardware.h"
#include "Sensor.h"
#include "OneWire.h"
#include "WireDevice.h"

typedef bool (*getSensorValue)(MeasureType, float &);

class WireBus
{
  private:
    static uint8_t sDeviceCount;
    static uint8_t sDeviceIndex;
    static WireDevice *sDevice[COUNT_1WIRE_CHANNEL]; // list of all used devices accross all BM
    static uint32_t sUnknownDeviceDelay;
    static void setDeviceParameter(OneWire *iDevice, uint16_t iParamIndex);

    OneWireDS2482 gOneWireBM;
    bool gForceSensorRead = true;

    void publishSensors(OneWireDS2482 *iBM);
    void processOneWire(bool iForce = false);
    void processUnknownDevices();
    void processSensor(sSensorInfo *cData, getSensorValue fGetSensorValue, MeasureType iMeasureType, float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber);

  public:
    static bool processNewIdCallback(OneWire *iOneWireSensor);
    static bool measureOneWire(MeasureType iMeasureType, float &eValue);

    WireBus();
    WireBus(uint8_t iI2cAddressOffset);
    ~WireBus();

    void loop();
    void setup();
    void processKOCallback(GroupObject &iKo);
};
