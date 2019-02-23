
/************************************
 * LEAF Branch v.1 
 * Designed for Particle Argon  
 * Maintained by: Je'aime Powell
 * Contact: jeaimehp@gmail.com
 * Initally Coded: 02/13/19
 ***********************************/
 
/**********************************
  **** Sensors and Connections *****
  * ********************************
  * SD Datalogger (3.3v & GND) 
  * -- Adalogger FeatherWing 
  *     - SCK  - D13 [SCK]
  *     - MOSI - D12 [MOSI]
  *     - MISO - D11 [MISO]
  *     - SDCS - D5
  * 
  * RTC (3.3v & GND) 
  * -- Adalogger FeatherWing
  *     - SCL -  1 [SCL] *Not Used
  *     - SDA -  0 [SDA] *Not USed
  * 
  * Tipping Bucket (Reed Switch) (3.3v)
  *     - D4 - INPUT_PULLUP pinmode
  * 
  * Relay Channel 1 - 3.3v
  *     - SIG - D6
  * 
  * 
  * Thermocouple (3.3v & GND) 
  * --(T-Type Thermocouple - Copper Constantant)
  * --Adafruit MAX31856
  *     - CK  - D13 [SCK]
  *     - SDI - D12 [MOSI]
  *     - SDO - D11 [MISO]
  *     - CS - A5 [SS]
  * 
  * Accelerometer/Gyroscope/Magnetometer (3.3v & GND)
  * -- HiLetgo MPU-9250 - 0x68
  *     - SCL -  1 [SCL] 
  *     - SDA -  0 [SDA]
  * 
  * Temperature/Humidity/Pressure (3.3v & GND
  * -- BME280 - 0x76
  *     - SCL -  1 [SCL] 
  *     - SDA -  0 [SDA]
  * ******************************/


///////////////////////////
// Libraries
///////////////////////////

// This #include statement was automatically added by the Particle IDE.
#include <MAX31856TC.h>

// This #include statement was automatically added by the Particle IDE.
 #define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunBME280.h>

// This #include statement was automatically added by the Particle IDE.
//#include <SparkFun_MPU-9250.h>
 #include <MPU9250.h>

// SD Card Libraries
#include <SPI.h>
#include <SdFat.h>
//-------------------------------------------------------//


 
///////////////////////////
// Sensor and Pin Definitions
///////////////////////////
 
//Tipping Bucket 
    #define DRIP_SENSOR_DETECT D4
    #define DRIP_SENSOR_OUT D3
 
// Relay Channel 1 (3.3v Power to Sensors)
    #define power3_3v D6
 

// MAX31856 Thermocouple
    #define CS1  A5 //CS of Thermocouple ADC


    #define CR0_INIT 0x88 // Linearized Temperature Low Fault Threshold LSB
    // Thermocouple supported types found in MAX31856TC Library .h file
    #define CR1_INIT  0x07 // T-Type Therocouple 
    #define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))
    
    MAX31856TC* p1;

    
// MPU9250 (Accl/Gyro/Mag) Specific Definitions
    MPU9250 myIMU;
// T/RH/P BME280 I2C Definition
    BME280 t_rh_p_bme;

// Adalogger SDCArd CS 
    // SD chip select pin
    const uint8_t chipSelect = D5;

 
//-------------------------------------------------------//
 
 
///////////////////////////
//Leaf Sensor Class
///////////////////////////
class leafSENSORS{
     private:
        // Variables local to class
        
     public:
        // Sensor Function Calls
        
        
        //-------BME280 Temperature----------//
        float bme_temp(){
            t_rh_p_bme.readTempC();
            return t_rh_p_bme.readTempC();
        }
         //-------BME280 Relative Humidity--//
        float bme_rh(){
            return t_rh_p_bme.readFloatHumidity();
        }
         //-------BME280 Pressure----------//
        float bme_p(){
            return t_rh_p_bme.readFloatPressure();
        }
         //-------BME280 Altitude----------//
        double bme_altitude(){
            return t_rh_p_bme.readFloatAltitudeFeet();
        }
        
        
};





///////////////////////////
// Global Objects
///////////////////////////
    leafSENSORS leaf;


    // Setting to automatically collect
    // data on power on 
    // 1 = Yes, do collect data
    // 0 = No, don't collect data
    int collect_data = 1;

// Tipping Bucket
    volatile int bucket_state = 0;
    const unsigned int DEBOUNCE_TIME = 1000;
    static unsigned long last_interrupt_time = 0;
    int rain = 0;

// Variables for Daily Time Sync 
// Ref: https://docs.particle.io/reference/device-os/firmware/xenon/#particle-synctime-
    #define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
    unsigned long lastSync = millis();
    
    String mydevID = System.deviceID();
    
// SD Card File Object
    // file system
    SdFat sd;

    // test file
    SdFile myFile;

//-------------------------------------------------------//


///////////////////////////
// SETUP
///////////////////////////

void setup() {
    // Sensor Status JSON
   // DynamicJsonBuffer jBuffer;
    //JsonObject& jsonsensor = jBuffer.createObject();

    
    // Entering Setup console message
    Particle.publish("Entering Setup", "OK");
    //jsonsensor["SetupInit"] = "OK";
    delay(1000);
     //Pin Relays Control 3.3v and 5v to Sensors
    pinMode(power3_3v, OUTPUT);
    Particle.publish("Power Relays Setup", "OK");
    //jsonsensor["PowerRelay"] = "INITIALIZED";
    delay(2000);
    //Initialize BME280
    //I2C Setup
    Particle.publish("BME_Setup", "Initializing");
    delay(1000);
	t_rh_p_bme.settings.commInterface = I2C_MODE;
	t_rh_p_bme.settings.I2CAddress = 0x76;
	//***Operation settings*****************************//
	//runMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	t_rh_p_bme.settings.runMode = 3; //Normal mode
	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	t_rh_p_bme.settings.tStandby = 0;
	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	t_rh_p_bme.settings.filter = 4;
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.tempOverSample = 5;
	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    t_rh_p_bme.settings.pressOverSample = 5;
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.humidOverSample = 5;
	//Calling .begin() causes the settings to be loaded
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
//;
	if (!t_rh_p_bme.begin()){
        Particle.publish("BME_Setup", "BME Failed");
        Particle.publish("ALERT", "BME280_Failure!");
        //jsonsensor["BME280"] = "FAILED";
    }
    else{
        Particle.publish("BME_Setup", "OK"); 
        //jsonsensor["BME280"] = "OK";
    }
    
    
    //Initialize Rain Bucket Intterupt 
    pinMode(DRIP_SENSOR_DETECT, INPUT_PULLUP); // Interrupt pin
    //pinMode(DRIP_SENSOR_OUT, OUTPUT); // Power to Interrupt pin
    attachInterrupt(DRIP_SENSOR_DETECT, bucket_handler, FALLING);

   
    //Initialize MPU9250
    Particle.publish("MPU9250_Setup", "Initializing");
    myIMU.initMPU9250();
    Particle.publish("MPU9250_Setup", "OK");
    //jsonsensor["MPU9250"] = "INITIALIZED";

    //Initialize SD Card
    Particle.publish("SDCard Logger", "Initializing");
    delay(1000);
     if (!sd.begin(chipSelect)){
         Particle.publish("SDCard Logger", "Failed");
         //jsonsensor["SDCard"] = "FAILED";
         delay(1000);
         Particle.publish("ALERT", "SDCard_Failure!");
     }
     else {
            Particle.publish("SDCard Logger", "OK");
            //jsonsensor["SDCard"] = "OK";
            delay(1000);
    }



    // Initialize MAX31856
    // Define the pins used to communicate with the MAX31856
    p1 = new MAX31856TC(CS1);
    // Initializing the MAX31856's registers
    p1->writeRegister(REGISTER_CR0, CR0_INIT);
    p1->writeRegister(REGISTER_CR1, CR1_INIT);
    p1->writeRegister(REGISTER_MASK, MASK_INIT);
    
    // Ensures the Cold Junction is cleared
    digitalWrite(A5, LOW);
    SPI.transfer(0x0A);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    digitalWrite(A5, HIGH);
    Particle.publish("MAX31856TC", "Initialized");
    //jsonsensor["MAX318TC"] = "INITIALIZED";
  
    delay(200);
    
    
     
    ///////////////////////////////
    // Particle Publish Functions
    //////////////////////////////
    Particle.function("Power_3.3v",relay33v);
    Particle.function("Data_ONOFF",data_setting);
    Particle.function("Reset_Unit",resetunit);
    Particle.function("Uptime_s", sys_uptime);
    Particle.function("Device_Stat", dev_status);
    
    
    
    // Send Sitrep
    delay(1000);
    dev_status("phone home");
    
    

   
    //if (jsonsensor["SDCard" == "OK"]){
        //Set SPI to correct mode to write to the SD Card
        SPI.setDataMode(SPI_MODE0);
        // open the file for write at end like the "Native SD library"
        if (!myFile.open("startup.txt", O_RDWR | O_CREAT | O_AT_END)) {
            //sd.errorHalt("opening startup.log for write failed");
            Particle.publish("SDCard_Setup", "SD Card startup.txt init failed");
            Particle.publish("ALERT", "SD Card startup.txt init failed");
        }
        else {
              // if the file opened okay, write to it:
            Time.zone(0);
            time_t time = Time.now();
            String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);
            myFile.println("////////////////SD Card initialized///////////////////");
            myFile.printf(timeStamp,"\n");
            myFile.printf("fileSize: %d\n", myFile.fileSize());
            myFile.printf("Device ID ",mydevID,"\n");
            myFile.printf("[--------------------------------------------------]\n");
  
             // close the file:
            myFile.close();
        }
    //}
    
     // Exiting Setup console message
    delay(5000);
    Particle.publish("Exiting Setup", "OK");
    //jsonsensor["SetupExit"] = "OK";
}

//-------------------------------------------------------//

///////////////////////////
// LOOP
///////////////////////////

void loop() {
    // Daily Time Sync
    if (millis() - lastSync > ONE_DAY_MILLIS) {
        // Request time synchronization from the Particle Device Cloud
        Particle.syncTime();
        lastSync = millis();
        Particle.publish("Syncing Time", "OK");
        dev_status("phone home");
    }


    //Allows data collection to pause
    if (collect_data == 1){
    
        DynamicJsonBuffer jBuffer;
        JsonObject& jsondata = jBuffer.createObject();
        JsonObject& jsonmov = jBuffer.createObject();


        // Timestamp
        // Time.zone(-6);  // Central Time
        Time.zone(0);  // UTC - Needed for Chords
        time_t time = Time.now();
        String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);

        // Accelerometer 
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.getAres();

        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
        myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
        myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

        myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
        myIMU.getGres();

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;
    
        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;

        // Pulls the Thermocouple Temperature in C
         // Changes to the proper mode for the MAX31856
        SPI.setDataMode(SPI_MODE1);
        double sapfluxC = p1->readThermocouple(CELSIUS);

        // Formula to convert T-Type Thermocouple to mV
        double sapfluxmV = 0.00004*pow(sapfluxC, 2) + 0.0386*sapfluxC + 0.0007;

        //JSON Full Data 
        jsondata["bme_temp"] = leaf.bme_temp();
        jsondata["bme_rh"] = leaf.bme_rh();
        jsondata["bme_pressure"] = leaf.bme_p();
        jsondata["bme_altitude"] = leaf.bme_altitude();
        jsondata["rain_tip"] = bucket_state;
        // Bucket Count Reset
        bucket_state = 0;
        jsondata["imu_temp"] = myIMU.temperature;
        jsondata["sapfluxjc"] = sapfluxC;
        jsondata["sapfluxmv"] = sapfluxmV;
        jsonmov["at"] = timeStamp;
        jsondata["at"] = timeStamp;

        //JSON Movement Data 
        jsonmov["ax"] = myIMU.ax;
        jsonmov["ay"] = myIMU.ay;
        jsonmov["az"] = myIMU.az;
        jsonmov["gx"] = myIMU.gx;
        jsonmov["gy"] = myIMU.gy;
        jsonmov["gz"] = myIMU.gz;

        // fulldata event Creation
        String fulldata;
        jsondata.printTo(fulldata);
        Particle.publish("Full Data",fulldata);
        delay(1500);

        // movdata event creation
        String movdata;
        jsonmov.printTo(movdata);
        Particle.publish("Move Data",movdata);
        delay(1000);
        
        
        // SD Card Loggin of data
        //Set SPI to correct mode to write to the SD Card
        SPI.setDataMode(SPI_MODE0);
        String data_file_name = mydevID + Time.format(Time.now(), "%F") + ".log";
        if (!myFile.open(data_file_name, O_RDWR | O_CREAT | O_AT_END)) {
            Particle.publish("SDCard_Write", "SD Card data_file_name failed");
            Particle.publish("ALERT", "SD Card data_file_name init failed");
            
        }
        else{
            myFile.println(fulldata);
            myFile.println(movdata);
            myFile.close();
        }
        
        

        // Delay until next read
        delay(7000);
        //Reattaching Intterupt Post trip
        //attachInterrupt(DRIP_SENSOR_DETECT, bucket_handler, FALLING);
    } // End Data Collection "If" switch  

}

//-------------------------------------------------------//

//////////////////////////////////
// Published Particle Functions
//////////////////////////////////


int relay33v(String pow_state){
    Particle.publish("3.3v Power", pow_state);
    if (pow_state == "Reset"){
        digitalWrite(power3_3v, HIGH);
        Particle.publish("3.3v Power" , "Off");
        delay(200);
        digitalWrite(power3_3v, LOW);
        Particle.publish("3.3v Power" , "On");
         return 1;
        }
    else if (pow_state == "ON"){
        digitalWrite(power3_3v, LOW);
        Particle.publish("3.3v Power" , "On");
        return 1;
        }
    else if (pow_state == "OFF"){
        digitalWrite(power3_3v, HIGH);
        Particle.publish("3.3v Power" , "Off");
        return 1;
        }
    else{
        Particle.publish("Fail", "ON-OFF-Reset");
        return 0;
        }
    }


// Data Collection Function
int data_setting(String setting){
    if (setting == "0"){
        collect_data = 0;
        return 1;
        }
    else if (setting == "1"){
        collect_data = 1;
        return 1;
        }
    else {
        Particle.publish("Fail", "0-1");
        return 0;
        }
    }

// System Reset
int resetunit(String resetYN) {
    if (resetYN == "YES"){
      System.reset();
      return 1;
        }
    else {
      Particle.publish("Fail", "YES-to-reset");
      return 0;
        }
    }

// System Uptime
int sys_uptime(String blah){
    int uptime = System.uptime();
    return uptime;
}

int dev_status(String blah){
    delay(100);
    byte mac[6];
    WiFi.macAddress(mac);
    String stringmac;
    for (byte i = 0; i < 6; ++i)
        {
        char buf[3];
        sprintf(buf, "%2X", mac[i]);
        stringmac += buf;
        if (i < 5) stringmac += ':';
    }
    int uptime = System.uptime();
    // Create JSON Object
    DynamicJsonBuffer jBuffer;
    JsonObject& jsonstatus = jBuffer.createObject();
    // Create Wifi Object
    WiFiSignal sig = WiFi.RSSI();
    float strength = sig.getStrength();
    
    jsonstatus["Device_ID"] = mydevID;
    jsonstatus["Device_Ver"] = System.version();
    jsonstatus["Device_IP"] = String(WiFi.localIP());
    jsonstatus["Device_MAC"] = stringmac;
    jsonstatus["Device_Ut"] = uptime;
    jsonstatus["Wifi_SSID"] = WiFi.SSID();
    jsonstatus["Wifi_Sig"] = strength;
    
    
    //Create JSON String
    String sitrep;
    jsonstatus.printTo(sitrep);
    Particle.publish("SitRep",sitrep);
    return 1;
}


//-------------------------------------------------------//

//////////////////////////////
// ISR Handlers
/////////////////////////////

// Interrupt handler for drip bucket
void bucket_handler() {
    unsigned long interrupt_time = micros();
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
    bucket_state++;
    }
    last_interrupt_time = interrupt_time; 
    
    }


//-------------------------------------------------------//