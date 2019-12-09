#include <M5Stack.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2904.h>
#include <Adafruit_NeoPixel.h>
#include "utility/MPU9250.h"
#include <MadgwickAHRS.h>




class ServerCallbacks: public BLEServerCallbacks {
 public:
    bool* _pConnected;

    ServerCallbacks(bool* connected) {
      _pConnected = connected;
    }
    void onConnect(BLEServer* pServer) {
      *_pConnected = true;
      M5.Lcd.println("ServerCallbacks onConnect");
      Serial.println("ServerCallbacks onConnect");
    };
 
    void onDisconnect(BLEServer* pServer) {
      *_pConnected = false;
      M5.Lcd.println("ServerCallbacks onDisconnect");
      Serial.println("ServerCallbacks onDisconnect");
    }
};




std::string toMACAddrString(uint8_t* mac) {
  char buf[256];
  size_t len = sprintf(buf, " %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); 
  return std::string(buf, len);
};

std::string toUInt8String(uint8_t v) {
  char buf[256];
  size_t len = sprintf(buf, " %04d", v); 
  return std::string(buf, len);  
};

class InformationCallbacks: public BLECharacteristicCallbacks {
public:
  std::string _value;
  InformationCallbacks(std::string value) {
    _value = value;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    pCharacteristic-> setValue(_value);
  } 
};

BLECharacteristic* addInformationCharacteristic(BLEService *pService, uint16_t uuid, std::string value) {
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                      BLEUUID(uuid),
                      BLECharacteristic::PROPERTY_READ
                    );
  pCharacteristic->setCallbacks(new InformationCallbacks(value));  
  return pCharacteristic;
};

BLEService* addInformationService(BLEServer* pServer) {
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x180a));
  
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
   
//  addBinaryInformationCharacteristic(pService, (uint16_t)0x2a23, "System ID");
  addInformationCharacteristic(pService, (uint16_t)0x2a24, "M5Stack Fire");
  addInformationCharacteristic(pService, (uint16_t)0x2a25, toMACAddrString(mac));
  addInformationCharacteristic(pService, (uint16_t)0x2a26, esp_get_idf_version());
  addInformationCharacteristic(pService, (uint16_t)0x2a27, toUInt8String(ESP.getChipRevision()));
//  addInformationCharacteristic(pService, (uint16_t)0x2a28, "Software Rev");
  addInformationCharacteristic(pService, (uint16_t)0x2a29, "M5Stack");
  return pService;
};




class SimpleKeysCallbacks: public BLECharacteristicCallbacks {
public:
  BLECharacteristic* _pCharacteristic;
  SimpleKeysCallbacks(BLECharacteristic* pCharacteristic) {
    _pCharacteristic = pCharacteristic;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    uint8_t value = 0x00;
    value |= M5.BtnA.wasReleased() ? 0x01 : 0x00;
    value |= M5.BtnB.wasReleased() ? 0x02 : 0x00;
    value |= M5.BtnC.wasReleased() ? 0x04 : 0x00;
    pCharacteristic->setValue(&value, 1);
  }
  void update(boolean notify) {
    onRead(_pCharacteristic);
    uint8_t value = *_pCharacteristic->getData();
    if (notify) {
      _pCharacteristic->notify(); 
    } 
    if (value != 0x00) {
      String msg = "Button was released";
      M5.Lcd.println(msg);
      Serial.println(msg);
    }    
  }
};
  
SimpleKeysCallbacks *pSimpleKeysCallbacks;

BLEService* addSimpleKeysService(BLEServer* pServer) {
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0xffe0));

  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                        BLEUUID((uint16_t)0xffe1),
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
                      
  pSimpleKeysCallbacks = new SimpleKeysCallbacks(pCharacteristic); 
  pCharacteristic->setCallbacks(pSimpleKeysCallbacks);

  return pService;
};




class BatteryLevelCallbacks: public BLECharacteristicCallbacks {
public:
  BLECharacteristic* _pCharacteristic;
  BatteryLevelCallbacks(BLECharacteristic* pCharacteristic) {
    _pCharacteristic = pCharacteristic;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    uint8_t level = M5.Power.getBatteryLevel();
    _pCharacteristic->setValue(&level, 1);
  }
  void update(boolean notify) {
    onRead(_pCharacteristic);
    if (notify) {
      _pCharacteristic->notify();
    }
  }
};

BatteryLevelCallbacks *pBatteryLevelCallbacks;

BLEService* addBatteryService(BLEServer* pServer) {
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x180F));
  
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                      BLEUUID((uint16_t)0x2a19),
                      BLECharacteristic::PROPERTY_READ
                    );

  BLE2904 *pDescriptor = new BLE2904();
  pDescriptor->setFormat(BLE2904::FORMAT_UINT8);
  pDescriptor->setNamespace(1);
  pDescriptor->setUnit(0x27ad);                   
  pCharacteristic->addDescriptor(pDescriptor);

  pBatteryLevelCallbacks = new BatteryLevelCallbacks(pCharacteristic);
  pCharacteristic->setCallbacks(pBatteryLevelCallbacks);
  
  return pService;
};






#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15

Adafruit_NeoPixel Pixels = Adafruit_NeoPixel(
  M5STACK_FIRE_NEO_NUM_LEDS,
  M5STACK_FIRE_NEO_DATA_PIN,
  NEO_GRB + NEO_KHZ800);

class NeoPixelBrightnessCallbacks: public BLECharacteristicCallbacks {
public:
  BLECharacteristic* _pCharacteristic;
  NeoPixelBrightnessCallbacks(BLECharacteristic* pCharacteristic) {
    _pCharacteristic = pCharacteristic;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    uint32_t brightness = Pixels.getBrightness();
    _pCharacteristic->setValue(brightness);
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    const char *value = pCharacteristic->getValue().c_str();
    uint32_t* pbrightness = (uint32_t*)value;
    Pixels.setBrightness(*pbrightness);     
    Pixels.show();
  }
  void uodate(boolean notify) {
    onRead(_pCharacteristic);
    if (notify) {
      _pCharacteristic->notify();
    }    
  }
};

NeoPixelBrightnessCallbacks* pNeoPixelBrightnessCallbacks;

BLECharacteristic *pCharaNeoPixels[M5STACK_FIRE_NEO_NUM_LEDS];

class NeoPixelColorCallbacks: public BLECharacteristicCallbacks {
private:
  BLECharacteristic* _pCharacteristic;
  int _led;
public:
  NeoPixelColorCallbacks(BLECharacteristic* pCharacteristic, int led) {
    _pCharacteristic = pCharacteristic;
    _led = led;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
     uint32_t color = Pixels.getPixelColor(_led);
    _pCharacteristic->setValue(color);
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    const char *value = pCharacteristic->getValue().c_str();
    uint32_t* pcolor = (uint32_t*)value;
    Pixels.setPixelColor(_led, *pcolor);     
    Pixels.show();
  }
  void update(boolean notify) {
    onRead(_pCharacteristic);
    if (notify) {
      _pCharacteristic->notify();
    }
  }
};

NeoPixelColorCallbacks *pNeoPixelColorCallbacks[M5STACK_FIRE_NEO_NUM_LEDS];

#define neo_pixel  "209b0000-13f8-43d4-a0ed-02b2c44e6fd4"
#define brightness "209b0001-13f8-43d4-a0ed-02b2c44e6fd4"
#define led_color  "209b%04x-13f8-43d4-a0ed-02b2c44e6fd4"

BLEService* addNeoPixelService(BLEServer* pServer) {
  BLEService* pService = pServer->createService(neo_pixel);
  
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                        brightness,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE 
                        );
  pNeoPixelBrightnessCallbacks = new NeoPixelBrightnessCallbacks(pCharacteristic); 
  pCharacteristic->setCallbacks(pNeoPixelBrightnessCallbacks);

  for (int led=0; led<M5STACK_FIRE_NEO_NUM_LEDS; led++) {
    char uuid[256];
    sprintf(uuid, led_color, led + 2);
    pCharaNeoPixels[led] = pService->createCharacteristic(
                        uuid,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE 
                      );
                      
    pNeoPixelColorCallbacks[led] = new NeoPixelColorCallbacks(pCharaNeoPixels[led], led);
    pCharaNeoPixels[led]->setCallbacks(pNeoPixelColorCallbacks[led]);
  }
  return pService;
};





MPU9250 IMU;
Madgwick filter;

class MPU9250Callbacks: public BLECharacteristicCallbacks {
public:
  BLECharacteristic* _pCharacteristic;
  MPU9250Callbacks(BLECharacteristic* pCharacteristic) {
    _pCharacteristic = pCharacteristic;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
      IMU.readAccelData(IMU.accelCount);
      IMU.getAres();
      
      IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
      IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
      IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

      IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
      IMU.getGres();
     
      IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
      IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
      IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

      IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
      IMU.getMres();
/*
      IMU.magbias[0] = +470.;
      IMU.magbias[1] = +120.;
      IMU.magbias[2] = +125.;
*/
      IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] - IMU.magbias[0];
      IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] - IMU.magbias[1];
      IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] - IMU.magbias[2];

      filter.updateIMU(IMU.gx, IMU.gy, IMU.gz, IMU.ax, IMU.ay, IMU.az);
      IMU.roll = filter.getRoll();
      IMU.pitch = filter.getPitch();
      IMU.yaw = filter.getYaw();

      IMU.tempCount = IMU.readTempData();
      IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
      
      int16_t value[9];
      value[0] = (int16_t)(IMU.gx*65536/500);
      value[1] = (int16_t)(IMU.gy*65536/500);
      value[2] = (int16_t)(IMU.gz*65536/500);
      value[3] = (int16_t)(IMU.ax*32768/8);
      value[4] = (int16_t)(IMU.ay*32768/8);
      value[5] = (int16_t)(IMU.az*32768/8);
      value[6] = (int16_t)(IMU.mx/10*8190/32768);
      value[7] = (int16_t)(IMU.my/10*8190/32768);
      value[8] = (int16_t)(IMU.mz/10*8190/32768);      
      pCharacteristic->setValue((unsigned char*)value, sizeof(value));

      Serial.print(IMU.ax); Serial.print(", ");
      Serial.print(IMU.ay); Serial.print(", ");
      Serial.print(IMU.az); Serial.print(", ");
      Serial.print(IMU.gx); Serial.print(", ");
      Serial.print(IMU.gy); Serial.print(", ");
      Serial.print(IMU.gz); Serial.print(", ");
      Serial.print(IMU.mx); Serial.print(", ");
      Serial.print(IMU.my); Serial.print(", ");
      Serial.print(IMU.mz); 
      Serial.println("");
    }
  }
  void update(boolean notify) { 
      onRead(_pCharacteristic);
      if (notify) {
        _pCharacteristic->notify();
      }
  }
};

MPU9250Callbacks *pMPU9250Callbacks;

#define movement_sensor "f000aa80-0451-4000-b000-000000000000"
#define movement_value  "f000aa81-0451-4000-b000-000000000000"

BLEService* addMPU9250Service(BLEServer* pServer) {
  BLEService* pService = pServer->createService(movement_sensor);
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                      movement_value,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pMPU9250Callbacks = new MPU9250Callbacks(pCharacteristic);
  pCharacteristic->setCallbacks(pMPU9250Callbacks);

  return pService;
};



class IOCallbacks: public BLECharacteristicCallbacks {
public:
  BLECharacteristic* _pCharacteristic;
  IOCallbacks(BLECharacteristic* pCharacteristic) {
    _pCharacteristic = pCharacteristic;
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t value = pCharacteristic->getValue()[0];
    Serial.print(value);
    Serial.println("");
    bool red = (value & 0x01) > 0;
    if (red) {
      Pixels.setPixelColor(0, 0x00ff0000);
      Pixels.setPixelColor(1, 0x00ff0000);
      Pixels.setPixelColor(2, 0x00ff0000);
      Pixels.setPixelColor(3, 0x00ff0000);
      Pixels.setPixelColor(4, 0x00ff0000);  
      M5.Lcd.println("Red LED on");
      Serial.println("Red LED on");
    } else {
      Pixels.setPixelColor(0, 0x00000000); 
      Pixels.setPixelColor(1, 0x00000000); 
      Pixels.setPixelColor(2, 0x00000000); 
      Pixels.setPixelColor(3, 0x00000000); 
      Pixels.setPixelColor(4, 0x00000000);  
      M5.Lcd.println("Red LED off");
      Serial.println("Red LED off");         
    }
    bool green = (value & 0x02) > 0;
    if (green) {
      Pixels.setPixelColor(5, 0x0000ff00);
      Pixels.setPixelColor(6, 0x0000ff00);
      Pixels.setPixelColor(7, 0x0000ff00);
      Pixels.setPixelColor(8, 0x0000ff00);
      Pixels.setPixelColor(9, 0x0000ff00);
      M5.Lcd.println("Green LED on");
      Serial.println("Green LED on");      
    } else {
      Pixels.setPixelColor(5, 0x00000000);
      Pixels.setPixelColor(6, 0x00000000);
      Pixels.setPixelColor(7, 0x00000000);
      Pixels.setPixelColor(8, 0x00000000);
      Pixels.setPixelColor(9, 0x00000000); 
      M5.Lcd.println("Green LED off");
      Serial.println("Green LED off");         
    }
    bool buzzer = (value & 0x04) > 0;
    if (buzzer) {
      M5.Speaker.tone(440);
      M5.Lcd.println("Buzzer on");
      Serial.println("Buzzer on");      
    } else {
      M5.Speaker.mute();
      M5.Lcd.println("Buzzer off");
      Serial.println("Buzzer off");         
    }        
    Pixels.show();   
  }
};

#define io_service "f000aa64-0451-4000-b000-000000000000"
#define io_data    "f000aa65-0451-4000-b000-000000000000"
#define io_config  "f000aa66-0451-4000-b000-000000000000"

BLEService* addIOService(BLEServer* pServer) {
  BLEService* pService = pServer->createService(io_service);
  
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                      io_data,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new IOCallbacks(pCharacteristic));

  pCharacteristic = pService->createCharacteristic(
                      io_config,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new BLECharacteristicCallbacks());
  uint8_t mode = 0x01;
  pCharacteristic->setValue(&mode, 1);
                    
  return pService;
};

bool deviceConnected = false;

void setup() {
  M5.begin();

  M5.Lcd.println("enter setup");
  Serial.println("enter setup");

  BLEDevice::init("M5Stack Fire");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks(&deviceConnected));

  BLEService *pService = addInformationService(pServer);
  pService->start();
  M5.Lcd.println("Information Service start");
  Serial.println("Information Service start");

  M5.Power.begin();
  pService = addBatteryService(pServer);
  pService->start();
  M5.Lcd.println("Battery Service start");
  Serial.println("Battery Service start");

  Pixels.begin();
  Pixels.show();
  pService = addSimpleKeysService(pServer);
  pService->start();
  M5.Lcd.println("SimpleKeys Service start");
  Serial.println("SimpleKeys Service start");

  pService = addNeoPixelService(pServer);
  pService->start();
  M5.Lcd.println("NeoPixel Service start");
  Serial.println("NeoPixel Service start");

  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);
  filter.begin(1);
  pService = addMPU9250Service(pServer);
  pService->start();
  M5.Lcd.println("MPU9250 Service start");
  Serial.println("MPU9250 Service start");

  pService = addIOService(pServer);
  pService->start();
  M5.Lcd.println("IO Service start");
  Serial.println("IO Service start");  


  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  M5.Lcd.println("Advertising start");
  Serial.println("Advertising start");
  
  M5.Lcd.println("exit setup");
  Serial.println("exit setup");
}

void loop() {
  //Serial.println("enter loop");

  if (deviceConnected) {
    //Serial.printf("notify %d\n", level);
    
    pBatteryLevelCallbacks->update(true);
    pSimpleKeysCallbacks->update(true);

    pMPU9250Callbacks->update(true);
  }
  delay(300);
  M5.update();
}