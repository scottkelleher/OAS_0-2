////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Code Function: Alfa 0.2  (7-26-16, JS)
//  1 Test the Sharp, solar panals, BME280, and sd logging, to get first feild data
//  2 Test Manual mode, for power consumption and interfearence 
//  3 Test Over the Air Cominucation for data colection 
//  4 Test Google script alerts and logging
//
//
//To Do
//*Add multiple file names AOSXXX.txt
//*Add upload to Google Drive/publish variables 
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////
///////////////////////INA219////////////////////////{
/////////////////////////////////////////////////////
#include "adafruit-ina219/adafruit-ina219.h"
Adafruit_INA219 ina219;
/////////////////////////////////////////////////////
////////////////////////INA219///////////////////////}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
///////////////////Fuel gauge////////////////////////{
/////////////////////////////////////////////////////
#include "SparkFunMAX17043/SparkFunMAX17043.h"
double battery_volts = 0; // Variable to keep track of LiPo voltage
//double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
//bool alert; // Variable to keep track of whether alert has been triggered
/////////////////////////////////////////////////////
////////////////fuel gauge///////////////////////////}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
////////////////////////////SD///////////////////////{
/////////////////////////////////////////////////////
#include "SdFat/SdFat.h"
#define SPI_CONFIGURATION 0
//------------------------------------------------------------------------------
// Setup SPI configuration.
#if SPI_CONFIGURATION == 0
// Primary SPI with DMA
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFat sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 1
// Secondary SPI with DMA
// SCK => D4, MISO => D3, MOSI => D2, SS => D1
SdFat sd(1);
const uint8_t chipSelect = D1;
#elif SPI_CONFIGURATION == 2
// Primary SPI with Arduino SPI library style byte I/O.
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFatLibSpi sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 3
// Software SPI.  Use any digital pins.
// MISO => D5, MOSI => D6, SCK => D7, SS => D0
SdFatSoftSpi<D5, D6, D7> sd;
const uint8_t chipSelect = D0;
#endif  // SPI_CONFIGURATION

File myFile;
/////////////////////////////////////////////////////
////////////////////SD///////////////////////////////}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
////////////////////////////BME//////////////////////{
/////////////////////////////////////////////////////
#include "SparkFunBME280/SparkFunBME280.h"

BME280 mySensor;

unsigned int sampleNumber = 0; //For counting number of CSV rows
/////////////////////////////////////////////////////
////////////////////////////BME//////////////////////}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
////////////////////////////ads//////////////////////{
/////////////////////////////////////////////////////
#include "Adafruit_ADS1X15/Adafruit_ADS1X15.h"

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
double multiplier;
/////////////////////////////////////////////////////
////////////////////////////ads//////////////////////}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
/////////////////////////Variabels///////////////////{
/////////////////////////////////////////////////////
unsigned long previousMillis = 0;        // will store last time LED was updated

const int nAverage = 30;        //How many readings should be averaged 0<X<512
float sharp_readings[nAverage]; //
int reading_index=0; 
float sharp_a=0;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;


float batter_voltage = 0;   //Published
float tempC = 0;            //Published
float GPS_lon = 123.456;    //Published (WIP)
float GPS_lat = 123.456;    //Published (WIP)
float sharp_reading = 0; 
//Published


FuelGauge fuel;

///////////////////////////////////////////////
//////////////Particle Functions///////////////{
///////////////////////////////////////////////
    //Update these when you want to publish new data
char sharp_str[64];     
char battery_str[64];   
char GPS_str[64];
char temp_str[64];


unsigned long sharp_timer_publish = micros();
unsigned long five_minute_publish = micros();

//Controle variable of what to publish and how offent to publish 
int print_controle = 0x001D;
/*  0000 0000  0000 0000 = print nothing 
    XXXX XXXX  XXXX XX01 = print the sharp every five minutes
    XXXX XXXX  XXXX XX10 = print the sharp every two minutes 
    XXXX XXXX  XXXX XX11 = print the sharp every minute
    XXXX XXXX  XXXX X1XX = print the battery voltage every four minutes
    XXXX XXXX  XXXX 1XXX = print the GPS location every four minutes
    XXXX XXXX  XXX1 XxXX = print the temerature every four minutes
    XXXX XXXX  XX1X xxXX = print Something else
    XXXX XXXX  X1XX XxXX = print Something else...ECT
*/
///////////////////////////////////////////////
//////////////Particle Functions///////////////}
///////////////////////////////////////////////


/////////////////////////////////////////////////////
/////////////////////////Variables///////////////////}
/////////////////////////////////////////////////////

//SYSTEM_MODE(AUTOMATIC);
SYSTEM_MODE(MANUAL);
//Particle.connect();       //This will conect to the cloud
//Particle.process();       //This must then be called at least every 20s to maintain the conection

/////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    //while (!Serial) {     //wait for the Seial com port to be opened before running anything
    //    SysCall::yield();
    //}
    //RGB.control(true);
    //RGB.color(0,0,0);
    Time.zone(-6);
    Time.format(Time.now(), "%H:%M:%S");
    
    ///////////////////////////////////////////////
    //////////////Particle Functions///////////////{
    ///////////////////////////////////////////////
    //Variables 12 char MAX
    Particle.variable("sharp", sharp_str, STRING);  //Both the sharp and battery in one
    //Particle.variable("sharp", sharp_str, STRING);      //Sharp reading
    Particle.variable("battery", battery_str, STRING);  //Battery Voltage
    //Particle.variable("GPS", GPS_str, STRING);          //GPS location
    Particle.variable("temp", temp_str, STRING);        //Tempiture (Inside or Outside, IDK)
    
    //Particle.function("Pumps", Pump);                   //turn the pumps on and off
    Particle.function("Prints", print);                 //Determin what to print and print it
    
    print("command");
    ///////////////////////////////////////////////
    //////////////Particle Functions///////////////}
    ///////////////////////////////////////////////
    
    /////////////////////////////////////////////
    /////////////////GPIO////////////////////////{
    /////////////////////////////////////////////
    pinMode(D6, OUTPUT);
    digitalWrite(D6, HIGH);
    //pinMode(D5, OUTPUT);
    //digitalWrite(D5, HIGH);
    //pinMode(D6, OUTPUT);
    //digitalWrite(D6, HIGH);
    pinMode(D4, OUTPUT);
    digitalWrite(D4, LOW);
    
    pinMode(D7, OUTPUT); //Debug for pumps only
    digitalWrite(D7,LOW);
    /////////////////////////////////////////////
    ////////////////GPIO/////////////////////////}
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////
    ////////////////////SD///////////////////////{
    /////////////////////////////////////////////
    // Initialize SdFat or print a detailed error message and halt
    // Use half speed like the native library.
    // Change to SPI_FULL_SPEED for more performance.
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
        sd.initErrorHalt();
    }
    if (!myFile.open("AOS_Test.txt", O_RDWR | O_CREAT | O_AT_END)) {
        sd.errorHalt("opening test.txt for write failed");
    }
    
    myFile.close();
    /////////////////////////////////////////////
    ////////////////////SD///////////////////////}
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////
    ////////////////////BME//////////////////////{
    /////////////////////////////////////////////
    //***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x77;

	//For SPI enable the following and dissable the I2C section0
	//mySensor.settings.commInterface = SPI_MODE;
	//mySensor.settings.chipSelectPin = 10;


	//***Operation settings*****************************//
	mySensor.settings.runMode = 3; //  3, Normal mode
	mySensor.settings.tStandby = 0; //  0, 0.5ms
	mySensor.settings.filter = 0; //  0, filter off
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;
	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;
	
	mySensor.begin();
    /////////////////////////////////////////////
    ////////////////////BME//////////////////////}
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////
    ////////////////////ads//////////////////////{
    /////////////////////////////////////////////
    //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit =       3mV       0.1875mV (DEFAULT)
    //multiplier =  0.1875F;
    //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit =     2mV       0.125mV
    //multiplier =  0.125F;
    //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit =     1mV       0.0625mV
    //multiplier =  0.0625F;
    //ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit =     0.5mV     0.03125mV
    //multiplier =  0.03125F;
    //ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit =     0.25mV    0.015625mV
    //multiplier =  0.015625F;
    //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit =     0.125mV   0.0078125mV  
    //multiplier =  0.0078125F;
    //ads.begin();
    /////////////////////////////////////////////
    ////////////////////ads//////////////////////}
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////
    /////////////////fuel gauge//////////////////{
    /////////////////////////////////////////////

    lipo.begin(); // Initialize the MAX17043 LiPo fuel gauge

	// Quick start restarts the MAX17043 in hopes of getting a more accurate
	// guess for the SOC.
	//lipo.quickStart();

	// We can set an interrupt to alert when the myFile.print(",");batter SoC gets too low.
	// We can alert at anywhere between 1% - 32%:
    //lipo.setThreshold(20); // Set alert threshold to 20%.
    /////////////////////////////////////////////
    ////////////// fuel gauge ///////////////////}
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////
    ////////////////////ads115///////////////////{
    /////////////////////////////////////////////
    //uint32_t currentFrequency;
    ina219.begin();
    /////////////////////////////////////////////
    /////////////////ads115//////////////////////}
    /////////////////////////////////////////////
    
    
}
/////////////////////////////////////////////


void log_data() {
    //double sharp0, sharp1, sharp2, sharp3;
    //sharp0 = read_sharp(0);
    double particle_analogsignal;
    particle_analogsignal=voltageRaw();

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;

    float sharp_a = read_sharp_averaged();
   
   
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  

    
    //sharp1 = read_sharp(1);
    //sharp2 = read_sharp(2);
    //sharp3 = read_sharp(3);.
    
    float tempC, press, altM, RH;
    tempC = mySensor.readTempC();
    press = mySensor.readFloatPressure();
    altM  = mySensor.readFloatAltitudeMeters();
    RH    = mySensor.readFloatHumidity();
    
    if (!myFile.open("AOS_Test.txt", O_RDWR | O_AT_END)) {
        sd.errorHalt("opening test.txt for write failed");
    }
    
    myFile.print(tempC, 3); myFile.print(",");
    myFile.print(press, 1); myFile.print(",");
    //myFile.print(altM,  3); myFile.print(",");
    myFile.print(RH,    3); myFile.print(",");
    //myFile.print(sharp0,3); myFile.print(",");
    //myFile.print(particle_analogsignal,3);myFile.print(",");
    //myFile.print(sharp1,3); myFile.print(",");
   // myFile.print(sharp2,3); myFile.print(",");
    //myFile.print(sharp3,3); myFile.println(",");
    myFile.print(sharp_a,1); myFile.print(",");
    myFile.print(String::format( "%02.2f", fuel.getVCell())); myFile.print(",");
    myFile.print(busvoltage,2); myFile.print(",");
    myFile.print(shuntvoltage,2); myFile.print(",");
    myFile.print(loadvoltage,2); myFile.print(",");
    myFile.print(current_mA,3); myFile.print(",");
    
   
    myFile.print(String::format("%d-", Time.year()));
    myFile.print(String::format("%d-", Time.month()));
    myFile.print(String::format("%d,", Time.day()));
    myFile.print(String::format("%d:", Time.hour()));
    myFile.print(String::format("%d:", Time.minute()));
    myFile.println(String::format("%d", Time.second()));
   // myFile.print(battery_volts,3);myFile.print(",");
   // myFile.println(soc,3);
    myFile.close();
    
    
}

float voltageRaw (){
    digitalWrite(D6, 0);
    delayMicroseconds(280);
    sharp_reading = analogRead(0);
    delayMicroseconds(40);
    digitalWrite(D6, 1);
    
    //delay(12);
    return (sharp_reading);
}

float read_sharp_averaged(){
    reading_index++;
    if(reading_index >= nAverage){
        reading_index = 0;
    }
    sharp_readings[reading_index] = sharp_reading;
    //Serial.print(String::format("Index:%d Value:%f,", reading_index, sharp_readings[reading_index]));
    float sum  = 0;
    for(int i = 0; i < nAverage; i++){
        sum = sum + sharp_readings[i];
    }
    return sum/(nAverage);
}





void loop() {
    const unsigned long interval_1 = 5000;            // interval at which to collect data (milliseconds)
    const unsigned long interval_2 = 60000;           // interval at which to publish data (milliseconds)
    
    
    if(millis() - previousMillis >= interval_1) {
        previousMillis = millis();   
        log_data();
        
        float raw=voltageRaw();
        //Average();
        tempC = mySensor.readTempC();
        
        float sharp_a = read_sharp_averaged();
        //Serial.print(String::format("%f, %f,", tempC, sharp_a));
        //Serial.print(Time.timeStr());
        //Serial.print(String::format("%d-", Time.year()));
        //Serial.print(String::format("%d-", Time.month()));
        //Serial.print(String::format("%d,", Time.day()));
        //Serial.print(String::format("%d:", Time.hour()));
        //Serial.print(String::format("%d:", Time.minute()));
        //Serial.println(String::format("%d", Time.second()));
        
        FuelGauge fuel;
        //Serial.print(String::format( "%02.2f %03.2f", fuel.getVCell(), fuel.getSoC()));
        
        float shuntvoltage = 0;
        float busvoltage = 0;
        float current_mA = 0;
        float loadvoltage = 0;
        
        batter_voltage = fuel.getVCell();
        
        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        loadvoltage = busvoltage + (shuntvoltage / 1000);
        
        Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
        Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
        Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
        Serial.println("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    }
    
    
    push_new_data();   //Loop this continuesly and it will publish the data as needed (Don't comment out if you want to publish anything)
    delay(10);         //Optional delay
    
}




///////////////////////////////////////////////
//////////////Particle Functions///////////////{
///////////////////////////////////////////////

int print(String command){
    ////////////////////////////////////////////////////////////////////////
    // add bit that reads command and converts it into print_controle here
    ////////////////////////////////////////////////////////////////////////
    //if(command == "GPS"){    //print the GPS location
        char GPS_temp[32];
        int GPS_lon1 = (GPS_lon - (int)GPS_lon) * 10000;
        int GPS_lat1 = (GPS_lat - (int)GPS_lat) * 10000;
        sprintf(GPS_temp,"%0d.%d,%0d.%d", (int)GPS_lon, GPS_lon1, (int)GPS_lat, GPS_lat1);
        //google docs will get this variable - if you are storing the value in a google sheet
        sprintf(GPS_str, "{\"t\":%s}", GPS_temp);
    //}
    //if(command == "temp"){    //print the temerature
        char temp_temp[32];
        int temp_reading1 = (tempC - (int)tempC) * 100;
        sprintf(temp_temp,"%0d.%d", (int)tempC, temp_reading1);
        //google docs will get this variable - if you are storing the value in a google sheet
        sprintf(temp_str, "{\"t\":%s}", temp_temp);
    //}
    push_new_data();
}

int push_new_data(){
    switch(print_controle | 0x0003){                        //This may not be working corectly??? IDK
        case 2:     //print every two minutes
            if(micros() > sharp_timer_publish + 120000){
                publish_sharp();
            }
            break;
        case 3:     //print every minute
            if(micros() > sharp_timer_publish + 60000){
                publish_sharp();
            }
            break;
        default:
            if(micros() > sharp_timer_publish + 300000){
                publish_sharp();
            }
            break;
    }
    if(micros() > five_minute_publish + 300000){
        five_minute_publish = micros();
        //anything needing publishing every five minutes can go here
        print("command");   // Temporary update GPS and temp every five minutes
    }
    
}

void publish_sharp(){       //Publish sharp and battery state
    sharp_timer_publish = micros();
    
    char sharp_temp[32];
    int sharp_reading1 = (sharp_reading - (int)sharp_reading) * 100;
    sprintf(sharp_temp,"%0d.%d", (int)sharp_reading, sharp_reading1);
    
    char battery_temp[32];
    int battery_reading1 = (batter_voltage - (int)batter_voltage) * 100;
    sprintf(battery_temp,"%0d.%0", (int)batter_voltage, battery_reading1);
    
    //google docs will get this variable - if you are storing the value in a google sheet
    sprintf(sharp_str, "{\"t\":%s,%s}", sharp_temp, battery_temp);
    
    if (Particle.connected() == false) {
        Particle.connect();
    }
    Particle.process();
    
}


int Pump(String command){      //turn the pumps on and off
    if(command == "ON"){
        digitalWrite(D4, HIGH);
        digitalWrite(D7, HIGH); //Debug only
    }else{
        digitalWrite(D7, HIGH); //Debug only 
        delay(1000);
        digitalWrite(D4, LOW);
        digitalWrite(D7, LOW); //Debug only
    }
    
}


///////////////////////////////////////////////
//////////////Particle Functions///////////////}
///////////////////////////////////////////////




