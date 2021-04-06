#include "knx.h"
#include "Hardware.h"
#include "Sensor.h"
#include "OneWire.h"
#include "WireDevice.h"
#include "OneWireDS2482.h"

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
    bool gIsSetup = false;

    void publishSensors(OneWireDS2482 *iBM);
    void processOneWire(bool iForce = false);
    void processUnknownDevices();

  public:
    static bool processNewIdCallback(OneWire *iOneWireSensor);
    static void loopCallback(void *iThis);
    static void knxLoopCallback(); // just to avoid knx reference in common
    static bool measureOneWire(MeasureType iMeasureType, float &eValue);
    static void processIButtonGroups();

    WireBus();
    ~WireBus();

    void loop();
    void setup(uint8_t iI2cAddressOffset, bool iSearchNewDevices, bool iSearchIButtons);
    void processKOCallback(GroupObject &iKo);
};
