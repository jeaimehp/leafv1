// This #include statement was automatically added by the Particle IDE.
#include <MAX31856TC.h>

// This #include statement was automatically added by the Particle IDE.
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunBME280.h>



// This #include statement was automatically added by the Particle IDE.
#include <MPU9250.h>



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
  * Relay Channel 2 - 5v
  *     - SIG - D7
  * 
  * Thermocouple (3.3v & GND)
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
//-------------------------------------------------------//
 
 
 // Allow system to start Collecting Without WiFi
 //SYSTEM_THREAD(ENABLED)
 
///////////////////////////
// Libraries
///////////////////////////

// This #include statement was automatically added by the Particle IDE.


 
//-------------------------------------------------------//
 
///////////////////////////
// Sensor and Pin Definitions
///////////////////////////
 
//Tipping Bucket 
#define DRIP_SENSOR_PIN D4
 
// Relay Channel 1 (3.3v Power to Sensors)
#define power3_3v D6
 
// Relay Channel 1 (3.3v Power to Sensors)
#define power_5v D7

// MAX31856 Thermocouple
    #define CS1  A5

    // The default noise filter is 60Hz, suitable for the USA
    #define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K /* + CR0_NOISE_FILTER_50HZ */)
    #define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
    #define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))
    
    MAX31856TC* p1;
    MAX31856TC* p2;
    
// MPU9250 (Accl/Gyro/Mag) Specific Definitions
MPU9250 myIMU;
// T/RH/P BME280 I2C Definition
BME280 t_rh_p_bme;
 
//-------------------------------------------------------//
 
 
///////////////////////////
//Leaf Sensor Class
///////////////////////////
class leafSENSORS{
     private:
        // Variables local to class
        //MPU9250 (Accl/Gyro/Mag)
        float
            gx,gy,gz,
            ax,ay,az,
            mx,my,mz;
        //double t_bme, rel_humid_bme, p_bme;
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
volatile bool bucket_state = false;
const unsigned int DEBOUNCE_TIME = 750;
static unsigned long last_interrupt_time = 0;
int rain = 0;

///////////////////////////
// SETUP
///////////////////////////

void setup() {
    // Wait for Wifi

    // Entering Setup console message
    Particle.publish("Entering Setup", "OK");
    delay(1000);
     //Pin Relays Control 3.3v and 5v to Sensors
    pinMode(power3_3v, OUTPUT);
    pinMode(power_5v, OUTPUT);
    Particle.publish("Power Relays Setup", "OK");
    delay(1000);
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
	//Particle.publish("BME_Setup", "BME Failed");
	if (!t_rh_p_bme.begin()){
        Particle.publish("BME_Setup", "BME Failed");
    }
    else{
        Particle.publish("BME_Setup", "OK");    
    }
    
    
    //Initialize Rain Bucket Intterupt 
    pinMode(DRIP_SENSOR_PIN, INPUT_PULLUP); // Interrupt pin 
    //attachInterrupt(DRIP_SENSOR_PIN, bucket_handler, RISING);
    //Particle.variable("BucketCount", rain);
   
    //Initialize MPU9250
    Particle.publish("MPU9250_Setup", "Initializing");
    myIMU.initMPU9250();
    Particle.publish("MPU9250_Setup", "OK");

    // Initialize MAX31856
    // Define the pins used to communicate with the MAX31856
    p1 = new MAX31856TC(CS1);

  
    // Initializing the MAX31855's registers
    p1->writeRegister(REGISTER_CR0, CR0_INIT);
    p1->writeRegister(REGISTER_CR1, CR1_INIT);
    p1->writeRegister(REGISTER_MASK, MASK_INIT);
  
    delay(200);
    
    ///////////////////////////////
    // Particle Publish Functions
    //////////////////////////////
    Particle.function("Power_3.3v",relay33v);
    Particle.function("Power_5v",relay5v);
    Particle.function("Data_ONOFF",data_setting);

    // Exiting Setup console message
    delay(5000);
    Particle.publish("Exiting Setup", "OK");  
}

///////////////////////////
// LOOP
///////////////////////////

void loop() {
//Allows data collection to pause
// Increment and reset from interrupt
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME && bucket_state == true) {
       rain++;
       Particle.publish("Rain Detected",String(rain));
       bucket_state = false;
       last_interrupt_time = interrupt_time;
    }

if (collect_data == 1){
    
DynamicJsonBuffer jBuffer;
JsonObject& jsondata = jBuffer.createObject();


// Timestamp
Time.zone(-6);
time_t time = Time.now();
String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);



    

    
    // Accel
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



//JSON string creation
jsondata["bme_temp"] = leaf.bme_temp();
jsondata["bme_rh"] = leaf.bme_rh();
jsondata["bme_pressure"] = leaf.bme_p();
jsondata["bme_altitude"] = leaf.bme_altitude();
//jsondata["ax"] = myIMU.ax;
//jsondata["ay"] = myIMU.ay;
//jsondata["az"] = myIMU.az;
jsondata["gx"] = myIMU.gx;
jsondata["gy"] = myIMU.gy;
jsondata["gz"] = myIMU.gz;
jsondata["rain_tip"] = rain;
jsondata["imu_temp"] = myIMU.temperature;
jsondata["sapfluxjc"] = p1->readJunction(CELSIUS);
jsondata["sapfluxtc"] = p1->readThermocouple(CELSIUS);





String data = jsondata[String("bme_temp")];
String fulldata;
jsondata.printTo(fulldata); 
//jsondata.printTo(data);

//Particle.publish("Data",data, PRIVATE);
delay(1500);
Particle.publish("Full Data",fulldata);
if (rain > 0){
    rain = 0;
}



delay(15000);
} // End Data Collection if 

}

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

int relay5v(String pow_state){
    Particle.publish("5v Power", pow_state);
    if (pow_state == "Reset"){
        digitalWrite(power_5v, HIGH);
        Particle.publish("5v Power" , "Off");
        delay(200);
        digitalWrite(power_5v, LOW);
        Particle.publish("5v Power" , "On");
        return 1;
        }
    else if (pow_state == "ON"){
        digitalWrite(power_5v, LOW);
        Particle.publish("5v Power" , "On");
        return 1;
        }
    else if (pow_state == "OFF"){
        digitalWrite(power_5v, HIGH);
        Particle.publish("5v Power" , "Off");
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
        //noInterrupts();
        //detachInterrupt(DRIP_SENSOR_PIN);
        return 1;
    }
    else if (setting == "1"){
        collect_data = 1;
        //attachInterrupt(DRIP_SENSOR_PIN, bucket_handler, RISING);
        return 1;
    }
    else {
        Particle.publish("Fail", "0-1");
        return 0;
    }
}

// Interrupt handler for drip bucket
void bucket_handler() {
    bucket_state = true; 
}



