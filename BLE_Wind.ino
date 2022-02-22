/*
##################################################
#       ___  __   ____  _      ___         __    #
#      / _ )/ /  / __/ | | /| / (_)__  ___/ /    #
#     / _  / /__/ _/   | |/ |/ / / _ \/ _  /     # 
#    /____/____/___/   |__/|__/_/_//_/\_,_/      # 
#                                                #
##################################################  

  Name:    BLE_Wind
  Version: 0.1
  Date:    15 dec 2020
  Author:  Stéphane PENOT

  Hardware:
    Arduino Nano 33 BLE
    Calypso Ultrasonic Anemometer (BLE)
    OLED Display 128x32 I2C
  
  This software:
    -Automatic connection to the Calypso "ULTRASONIC"
    -Read the Calypso wind data
    -Calculate the pitch and roll with the LSM9DS1 IMU
    //-Correct the wind speed in function of the roll, pitch, roll speed, pitch speed
    -Display data on the OLED
      -Wind speed
      -Wind direction
      -Calypso Battery level
      -Temperature
      -Roll
      -Pitch
    -Transfert wind data on TX1    

  Automatic Reconnection
  Watchdog is activate for 30s

*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <SimpleKalmanFilter.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SERIALDEBUG 1
#define ECOMPASS 0

// ### BLE
#define CALYPSO_NAME "ULTRASONIC"
#define CALYPSO_ADDR "cd:fe:29:6e:da:87"
BLEDevice peripheral;
BLEService service_180d;

// WindSpeed...
// Principal Characteristic:  Notify
// Characteristic:            UUID: 0x2A39
BLECharacteristic DataCalypso_2a39;

// Data Rate Characteristic: Read/Write
// wind speed output data rate - 1hz,4hz,8hz
// UUID: 0xA002
// 0x01-> 1Hz
// 0x04-> 4Hz   >>> Default
// 0x08-> 8Hz
BLECharacteristic DataCalypso_a002;

// Sensors Characteristic: Read/Write
// Activate Clinometer and eCompass
// UUID: 0xA003
// 0x01-> ON
// 0x00-> OFF   >>> Default
BLECharacteristic DataCalypso_a003; // 

float Calypso_WindSpeed = 0;
int Calypso_WindDirection = 0;
int Calypso_BatteryLevel = 0;
int Calypso_Temperature = 0;
#if ECOMPASS == 1
int Calypso_Roll = 0;
int Calypso_Pitch = 0;
int Calypso_Compass = 0;
#endif

float Calypso_WindXComp = 0;
float Calypso_WindYComp = 0;
float CMWS = 0;
float CMWA = 0;
byte Calypso_DataRate = 0;

char Calypso_WindSpeedStr[15];
char Calypso_WindDirectionStr[15];
char Calypso_BatteryLevelStr[10];
char Calypso_TemperatureStr[10];
#if ECOMPASS == 1
char Calypso_RollStr[10];
char Calypso_PitchStr[10];
char Calypso_CompassStr[10];
#endif
char CMWSStr[15];
char CMWAStr[15];
char Calypso_DataRateStr[10];
// ### BLE

// ### OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ### OLED Display

// ### Roll and Pitch
#define OFFSET_GX  0
#define OFFSET_GY  0
#define OFFSET_GZ  0

#define OFFSET_AX -0.001538
#define OFFSET_AY -0.085525
//#define OFFSET_AZ  0.984127
#define OFFSET_AZ  0

float IMU_Pitch = 0;
float IMU_Roll = 0;
char IMU_RollStr[10];
char IMU_PitchStr[10];
// ### Roll and Pitch

#define X_MAT 2
#define Y_MAT 0
#define Z_MAT 10

#define MS_TO_KNOT 1.943844492

//#define DEGRES (char)223
#define DEGRES (char)247
#define PERCENT (char)37

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
SimpleKalmanFilter simpleKalmanFilterGX(0.5, 0.5, 0.01);
SimpleKalmanFilter simpleKalmanFilterGY(0.5, 0.5, 0.01);
SimpleKalmanFilter simpleKalmanFilterGZ(0.5, 0.5, 0.01);

SimpleKalmanFilter simpleKalmanFilterAX(0.1, 0.1, 0.1);
SimpleKalmanFilter simpleKalmanFilterAY(0.1, 0.1, 0.1);
SimpleKalmanFilter simpleKalmanFilterAZ(0.1, 0.1, 0.1);

//SimpleKalmanFilter simpleKalmanFilterWindXComp(0.1, 0.1, 0.01);
//SimpleKalmanFilter simpleKalmanFilterWindYComp(0.1, 0.1, 0.01);

//float estimated_valueWindXComp = 0;
//float estimated_valueWindYComp = 0;

//float estimated_valueWind_Speed = 0;
//float estimated_valueWind_Angle = 0;

//char estimated_valueWind_Speed_Str[15];
//char estimated_valueWind_Angle_Str[15];

// Serial output refresh time millisecond
const long SERIAL_REFRESH_TIME = 100;  // 0.5s
long Serial_refresh_time = 0;

// OLED output refresh time millisecond
const long OLED_REFRESH_TIME = 500;  // 0.5s
long OLED_refresh_time = 0;

// Calypso Update refresh time millisecond
const long CALYPSO_DATARATE_REFRESH_TIME = (30*1000);  // 30s
long Calypso_DataRate_refresh_time = 0;

// ### DataTransfert
char DataTr[200];
char NMEA_Data[200];
// ### DataTransfert

// ### Watchdog
// watchdog timeout in seconds
int wdt = 30;
// ### Watchdog

// Function

void(* resetFunc) (void) = 0;  //declare reset function @ address 0

void Connect() {
  // Connect to the BLE peripheral
  int nb=1;
#if SERIALDEBUG == 1
  Serial.println("Connecting...");
#endif
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(peripheral.localName());
  display.setCursor(0, 8);
  display.print(peripheral.address());
  display.setCursor(0, 16);
  display.print("Connecting...");
  display.display();

  while (peripheral.connect() == false) {
    // Watchdog
    // Reload the WDTs RR[0] reload register
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; 

#if SERIALDEBUG == 1
    Serial.println("Failed to connect!");
#endif
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(peripheral.localName());
    display.setCursor(0, 8);
    display.print(peripheral.address());
    display.setCursor(0, 16);
    display.print("Failed to connect! ");
    display.print(nb);
    display.display();
    
    delay(500);
    
    if (nb++ >= 10) {
      nb = 0;
      resetFunc();  //call reset;
    }
  }

  peripheral.discoverAttributes();
//        Serial.println(peripheral.serviceCount());
//
//        Serial.println(peripheral.advertisedServiceUuidCount());
//
  service_180d = peripheral.service("180d");
  
  DataCalypso_2a39 = service_180d.characteristic("2a39");
  DataCalypso_2a39.subscribe();  // Notification ubdate
  
  DataCalypso_a003 = service_180d.characteristic("a003");
#if ECOMPASS == 0
  DataCalypso_a003.writeValue((byte)0x00);  // Only Wind Speed, Wind Direction, Calypso_Temperature, Battery
#else
  DataCalypso_a003.writeValue((byte)0x01);  // with Roll, Pitch, eCompass
#endif

  DataCalypso_a002 = service_180d.characteristic("a002");
  DataCalypso_a002.readValue(Calypso_DataRate);

#if SERIALDEBUG == 1
  Serial.println("Connected...");
#endif
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(peripheral.localName());
  display.setCursor(0, 8);
  display.print(peripheral.address());
  display.setCursor(0, 16);
  display.print("Connected...");
  display.display();
}

void Calypso_WindXYComp() {
  Calypso_WindXComp = Calypso_WindSpeed * cos(Calypso_WindDirection * DEG_TO_RAD);
  Calypso_WindYComp = Calypso_WindSpeed * sin(Calypso_WindDirection * DEG_TO_RAD);
}

void setup() {
  
  // Watchdog  ### https://forum.arduino.cc/index.php?topic=643883.0
  //Configure WDT.
  NRF_WDT->CONFIG         = 0x01;             // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV            = wdt * 32768 + 1;  // set timeout
  NRF_WDT->RREN           = 0x01;             // Enable the RR[0] reload register
  NRF_WDT->TASKS_START    = 1;                // Start WDT

#if SERIALDEBUG == 0 
  Serial1.begin(19200);
#endif
  
#if SERIALDEBUG == 1
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");
#endif

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
#if SERIALDEBUG == 1    
    Serial.println(F("SSD1306 allocation failed"));
#endif
    for(;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Starting...");
  display.display();

  // Watchdog
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload; 

   // begin initialization
  if (!BLE.begin()) {
#if SERIALDEBUG == 1
    Serial.println("Starting BLE failed!");
#endif    
    display.setCursor(0, 8);
    display.print("Starting BLE failed!");
    display.display();
  
    while (1);
  }
#if SERIALDEBUG == 1
  Serial.println("BLE Central - Peripheral Explorer");
#endif
  display.setCursor(0, 8);
  display.print("BLE Explorer");
  display.display();

  // start scanning for peripherals
  BLE.scan();

  char Find = 0;
  
  while (Find == 0) {
    // Watchdog
    // Reload the WDTs RR[0] reload register
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; 
  
    // check if a peripheral has been discovered
    peripheral = BLE.available();
  
    if (peripheral) {
#if SERIALDEBUG == 1
      // discovered a peripheral, print out address, local name, and advertised service
      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
#endif  
      // see if peripheral is a ULTRASONIC
      if (peripheral.localName() == CALYPSO_NAME && peripheral.address() == CALYPSO_ADDR) {

        // stop scanning
        BLE.stopScan();

        Connect();
                
        Find=1;
      }
    }
  }

  if (!IMU.begin()) {
#if SERIALDEBUG == 1
    Serial.println("Failed to initialize IMU!");
#endif    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("IMU Init Failed!");
    display.display();
    
    while (1);
  }
}

void loop() {

  float Gx, Gy, Gz;
  float Ax, Ay, Az;
  float acc_total_vector;

  // Watchdog
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;  

  // Lecture du Gyro
  IMU.readGyroscope(Gx, Gy, Gz);

  // Offset Gyro
  Gx = Gx - OFFSET_GX;
  Gy = Gy - OFFSET_GY;
  Gz = Gz - OFFSET_GZ;

  // Filtrage Gyro
  float estimated_valueGX = simpleKalmanFilterGX.updateEstimate(Gx);
  float estimated_valueGY = simpleKalmanFilterGY.updateEstimate(Gy);
  float estimated_valueGZ = simpleKalmanFilterGZ.updateEstimate(Gz);


  // Lecture Accelero
  IMU.readAcceleration(Ax, Ay, Az);

  // Offset Accelero
  Ax = Ax - OFFSET_AX;
  Ay = Ay - OFFSET_AY;
  Az = Az - OFFSET_AZ;

  // Filtrage Accelero
  float estimated_valueAX = simpleKalmanFilterAX.updateEstimate(Ax);
  float estimated_valueAY = simpleKalmanFilterAY.updateEstimate(Ay);
  float estimated_valueAZ = simpleKalmanFilterAZ.updateEstimate(Az);
  
  // Vitesse de la tete de mat
  float MWx = (estimated_valueGY * DEG_TO_RAD * Z_MAT - estimated_valueGZ * DEG_TO_RAD * Y_MAT) * MS_TO_KNOT;
  float MWy = (-estimated_valueGX * DEG_TO_RAD * Z_MAT - estimated_valueGZ * DEG_TO_RAD * X_MAT) * MS_TO_KNOT;

  // Mesure angle Roll and Pitch
  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  acc_total_vector = sqrt(pow(estimated_valueAX, 2) + pow(estimated_valueAY, 2) + pow(estimated_valueAZ, 2));

  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(estimated_valueAX) < acc_total_vector) {
    IMU_Roll = asin(estimated_valueAY / acc_total_vector) * RAD_TO_DEG;
  }

  if (abs(estimated_valueAY) < acc_total_vector) {
    IMU_Pitch = asin(estimated_valueAX / acc_total_vector) * RAD_TO_DEG;
  }

  if (peripheral.connected()) {
    // while the peripheral is connected

    // check if the value of the simple DataCalypso_2a39 has been updated
    if (DataCalypso_2a39.valueUpdated()) {
      DataCalypso_2a39.read();
      extractData(DataCalypso_2a39.value(), DataCalypso_2a39.valueLength());

      Calypso_WindXYComp();

      // Filtrage du vent
/*    estimated_valueWindXComp = simpleKalmanFilterWindXComp.updateEstimate(Calypso_WindXComp);
      estimated_valueWindYComp = simpleKalmanFilterWindYComp.updateEstimate(Calypso_WindYComp);

      estimated_valueWind_Speed = sqrt(pow(estimated_valueWindXComp, 2) + pow(estimated_valueWindYComp, 2));

      estimated_valueWind_Angle = atan2(estimated_valueWindYComp, estimated_valueWindXComp) * RAD_TO_DEG;
      if (estimated_valueWind_Angle < 0) {
        estimated_valueWind_Angle = estimated_valueWind_Angle + 360;
      }
*/
 
      // Compensation du vent en fonction de la vitesse en tete de mat
      float CMWx = MWx - Calypso_WindXComp;
      float CMWy = MWy - Calypso_WindYComp;
    
      CMWS = sqrt(pow(CMWx, 2) + pow(CMWy, 2));
      CMWA = atan2(CMWy/cos(IMU_Roll * DEG_TO_RAD), CMWx) * RAD_TO_DEG;
      if (CMWA < 0) {
        CMWA = CMWA + 360;
      }
     
      // Transfert des data sur le port serie 1
      DataTransfert();
#if SERIALDEBUG == 0 
      Serial1.println(DataTr);
#endif
#if SERIALDEBUG == 1
      Serial.println(DataTr);
#endif    
    }
  }
  else {
    Connect(); // Reconnection
  }

  // Read Calypso_DataRate every 30 second
  if (millis() > Calypso_DataRate_refresh_time) {
    
    Calypso_DataRate = 0;
    DataCalypso_a002.readValue(Calypso_DataRate);

    if (Calypso_BatteryLevel > 30 && Calypso_DataRate <= 4) {
      DataCalypso_a002.writeValue((byte)0x08);
    }
  
    if (Calypso_BatteryLevel <= 30 && Calypso_DataRate == 8) {
      DataCalypso_a002.writeValue((byte)0x04);
    }

    DataCalypso_a002.readValue(Calypso_DataRate);

    Calypso_DataRate_refresh_time = millis() + CALYPSO_DATARATE_REFRESH_TIME;
  }

#if SERIALDEBUG == 1
  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > Serial_refresh_time) {
    Serial.print(Gx,4);
    Serial.print(",");
    Serial.print(estimated_valueGX,4);
    Serial.print(",");
    Serial.print(Gy,4);
    Serial.print(",");
    Serial.print(estimated_valueGY,4);
    Serial.print(",");
    Serial.print(Gz,4);
    Serial.print(",");
    Serial.print(estimated_valueGZ,4);
    Serial.print(",");
    
    Serial.print(Calypso_WindSpeed,2);
    Serial.print(",");
    Serial.print(Calypso_WindDirection,2);
    Serial.print(",");    
    Serial.print(MWx,4);
    Serial.print(",");
    Serial.print(MWy,4);

    Serial.print(",");
    Serial.print(Ax,4);
    Serial.print(",");
    Serial.print(estimated_valueAX,4);
    Serial.print(",");
    Serial.print(Ay,4);
    Serial.print(",");
    Serial.print(estimated_valueAY, 4);
    Serial.print(",");
    Serial.print(Az,4);
    Serial.print(",");
    Serial.print(estimated_valueAZ, 4);
   
    Serial.print(",");
    Serial.print(IMU_Roll, 4);
    Serial.print(",");
    Serial.print(IMU_Pitch, 4);

    Serial.print(",");
    Serial.print(acc_total_vector, 4);

    Serial.println();

    Serial_refresh_time = millis() + SERIAL_REFRESH_TIME;
  }
#endif

  // send to OLED output every 500ms
  if (millis() > OLED_refresh_time) {
    
    display_data();

    OLED_refresh_time = millis() + OLED_REFRESH_TIME;
  }
}

void extractData(const unsigned char data[], int length) {
    
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];
  }
  
  Calypso_WindSpeed     = (float)(data[1] * 256 + data[0]) / 100.0;
  Calypso_WindSpeed     = Calypso_WindSpeed * MS_TO_KNOT;
  Calypso_WindDirection = data[3] * 256 + data[2];
  Calypso_BatteryLevel  = data[4] * 10;
  Calypso_Temperature   = data[5] - 100;
#if ECOMPASS == 1
  Calypso_Roll          = data[6] - 90;
  Calypso_Pitch         = data[7] - 90;
  Calypso_Compass       = 360 - (data[9] * 256 + data[8]);
  if (Calypso_Compass==360) { Calypso_Compass = 0; } 
#endif
  
#if SERIALDEBUG == 1
//  //Wind Speed
//  Serial.print("Calypso Wind Speed: ");
//  Serial.println(Calypso_WindSpeed, 2);
//  //Wind Direction
//  Serial.print("Calypso Wind Direction: ");
//  Serial.println(Calypso_WindDirection, DEC);
//  //Battery level
//  Serial.print("Calypso Battery level: ");
//  Serial.println(Calypso_BatteryLevel, DEC);
//  //Calypso_Temperature Level
//  Serial.print("Calypso Temperature Level: ");
//  Serial.println(Calypso_Temperature, DEC);
//  
//  //Calypso Roll
//  Serial.print("Calypso Roll: ");
//  Serial.println(Calypso_Roll, DEC);
//  //Calypso Pitch
//  Serial.print("Calypso Pitch: ");
//  Serial.println(Calypso_Pitch, DEC);
//  //Calypso COMPASS
//  Serial.print("Calypso COMPASS: ");
//  Serial.println(Calypso_Compass, DEC);
//  
//
//  Serial.println();
#endif
}

void display_data() {
  // OLED Display
  display.clearDisplay();
  
  display.setCursor(0, 0);
  sprintf(Calypso_WindSpeedStr, "AWS:%4.1fk", Calypso_WindSpeed);
  display.print(Calypso_WindSpeedStr);
  
  display.setCursor(64, 0);
  sprintf(Calypso_WindDirectionStr, "AWD:%4d%c", Calypso_WindDirection, DEGRES);
  display.print(Calypso_WindDirectionStr);
 
  display.setCursor(0, 8);
  sprintf(CMWSStr, "CWS:%4.1fk", CMWS);
  display.print(CMWSStr);
  
  display.setCursor(64, 8);
  sprintf(CMWAStr, "CWD:%4d%c", (int)CMWA, DEGRES);
  display.print(CMWAStr);

  /*
  display.setCursor(0, 8);
  sprintf(estimated_valueWind_Speed_Str, "FWS:%4.1fk", estimated_valueWind_Speed);
  display.print(estimated_valueWind_Speed_Str);
  
  display.setCursor(64, 8);
  sprintf(estimated_valueWind_Angle_Str, "FWD:%4d%c", (int)estimated_valueWind_Angle, DEGRES);
  display.print(estimated_valueWind_Angle_Str);
  */
  display.setCursor(0, 16);
  sprintf(IMU_RollStr, "R: %5.1f%c", IMU_Roll, DEGRES); // IMU_Roll
  display.print(IMU_RollStr);

  display.setCursor(64, 16);
  sprintf(IMU_PitchStr, "P: %5.1f%c", IMU_Pitch, DEGRES); // IMU_Pitch
  display.print(IMU_PitchStr);
 
#if ECOMPASS == 0
  display.setCursor(0, 24);
  sprintf(Calypso_BatteryLevelStr, "Batt:%3d%c", Calypso_BatteryLevel, PERCENT);
  display.print(Calypso_BatteryLevelStr);
  
  display.setCursor(64, 24);
  sprintf(Calypso_TemperatureStr, "T:%3d%c", Calypso_Temperature, DEGRES);
  display.print(Calypso_TemperatureStr);

  display.setCursor(106, 24);
  sprintf(Calypso_DataRateStr, "%1dHz", Calypso_DataRate);
  display.print(Calypso_DataRateStr);
  
#else 
  
  display.setCursor(0, 24);
  sprintf(Calypso_RollStr, "R:%4.1f%c", Calypso_Roll, DEGRES); // Calypso_Roll
  display.print(Calypso_RollStr);

  display.setCursor(45, 24);
  sprintf(Calypso_PitchStr, "P:%4.1f%c", Calypso_Pitch, DEGRES); // Calypso_Pitch
  display.print(Calypso_PitchStr);

  display.setCursor(90, 24);
  sprintf(Calypso_CompassStr, "%4d%c", Calypso_Compass, DEGRES); // Calypso_Compass
  display.print(Calypso_CompassStr);
#endif


  display.display();
 
}

void DataTransfert() {
   int checksum;
  
  //sprintf(DataTr, "%4.1f,%4d,%4.1f,%4d,%5.1f,%5.1f,%3d", Calypso_WindSpeed, Calypso_WindDirection, CMWS, (int)CMWA, IMU_Roll, IMU_Pitch, Temperature);
//  sprintf(NMEA_Data, "CALRW,%f,%d,%f,%d,%f,%f,%d\0",
//                  Calypso_WindSpeed,
//                  Calypso_WindDirection,
//                  CMWS,
//                  (int)CMWA,
//                  IMU_Roll,
//                  IMU_Pitch,
//                  Calypso_Temperature);

  sprintf(NMEA_Data, "IIMWV,%.1f,R,%.1f,K\0",
                  (float)Calypso_WindDirection,
                  Calypso_WindSpeed);



  checksum = nmea0183_checksum(NMEA_Data);
  if (checksum <= 0x10) {
    sprintf(DataTr, "$%s*0%X", NMEA_Data, checksum);
  }
  else {
    sprintf(DataTr, "$%s*%X", NMEA_Data, checksum);
  }
                  
}

int nmea0183_checksum(char *nmea_data)
{
    int crc = 0;
    int i;

    for (i = 0; i < strlen(nmea_data); i ++) { // removed the - 3 because no cksum is present
        crc ^= nmea_data[i];
    }

    return crc;
}
