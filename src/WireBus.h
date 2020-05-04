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

    OneWireDS2482 gOneWireBM;
    bool gForceSensorRead = true;

    // temp: simulate KO which can be written
    uint8_t gOutput = 0;
    uint32_t gOutputDelay = 0;

    void setDeviceParameter(OneWire *iDevice, uint16_t iParamIndex);
    void publishSensors(OneWireDS2482 *iBM);
    void processOneWire(bool iForce = false);
    void processUnknownDevices();
    void processSensor(sSensorInfo *cData, getSensorValue fGetSensorValue, MeasureType iMeasureType, float iOffsetFactor, float iValueFactor, uint16_t iParamIndex, uint16_t iKoNumber);
    void simulateKO();

  public:
    static bool processNewIdCallback(OneWire *iOneWireSensor);
    static bool measureOneWire(MeasureType iMeasureType, float &eValue);

    WireBus();
    ~WireBus();

    void loop();
    void setup();
    void processKOCallback(uint16_t iKoIndex);
};
