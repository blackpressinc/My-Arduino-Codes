#include <Wire.h> //This library controls communication over i2c and other protocols that let you speak with sensors and other stuff. Very common in arduino/microcontroller world
#include <CD74HC4067.h> //A library for controlling the 16 channel multiplexer on the DAQ
#include <ADS1115_WE.h> //A library for controlling the analog to digital converter I chose for the DAQ
#include <SparkFun_TMP117.h> //A library for controlling the high accuracy temperature sensor. Unfortunately not on the current DAQ because #chipshortage
#include "BluetoothSerial.h" //Great library that makes sending/recieving text by bluetooth super easy (basically just use SerialBT instead of just Serial to send/recieve text
#include <OneWire.h> //A library for using the weird ass protocol DB18S20 temperature sensors use
#include <DallasTemperature.h> //A library for the DB18S20 sensors I added because I couldn't get the TMP117's

#define NumberOfThermocouples 4 /*I added this to get rid of unused channels. Currently it just starts reading the thermocouples from channel zero and it would go
0,1,2,3 if the "NumberOfThermocouples 4". So this means it wont print the higher channels. Eventually I want to replace this with an array that just calls out which 
channels are actually being used. Or better yet, some sort of code that does the same.*/

#define ONE_WIRE_BUS 21 //The DB18S20 data pin is connected to IO pin 21 for the ESP32 feather (the control board)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS); //This just sets the arduino to use the OneWire protocol on IO pin 21

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire); //Now that pin 21 is using OneWire, we let it know that it's communicating with a Dallas Temperature sensor on that channel

// arrays to hold device address
DeviceAddress insideThermometer; //Technically 64 DB18S20's can use the same pin 21 (that's why it's called "OneWire" because you can have a ton of sensors on one wire) so
//Soon we will scan for all "DallasTemperature" sensors on channel 21 and store their addresses in this array called insideThermometer

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) //If bluetooth for android is not already started, start bluetooth for android
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it //if bluetooth can't be run or is not enabled, throw a warning 
#endif

BluetoothSerial SerialBT; //Open a "BluetoothSerial" connection, i.e. a generic text connection via bluetooth that can be called by saying "SerialBT". You'll see it later on in my code

CD74HC4067 my_mux(19,18,5,4); //The multiplexer used 4 IO pins to control its state. I desperately want to switch this to i2c but couldn't find the chips. 

#define I2C_ADDRESS 0x48 //The ADS1115 chip could be on one of 4 i2c addresses but on my thermocouple DAQ it's set to 0x48
ADS1115_WE adc(I2C_ADDRESS); //This lets the arduino know that i2c address 0x48 is an ADS1115 sensor and we can call it by saying "adc" from now on in the code

TMP117 tmp117; //Create an instance fo the tmp117 which by default is on the i2c address 0x49 on my DAQ (but again, doesn't exist because chip shortage)

double TC_Voltage[NumberOfThermocouples]; //This is a placeholder array where I will dump the analog voltages read from the thermocouples later

//This is an array of calibration data from NIST for E-type thermocouples from 0 °C - 1000 °C. It converts temperatures to voltages
double TypeETemptoVolt[] = 
{
  0.000000000000E+00,
  0.586655087100E-01,
  0.450322755820E-04,
  0.289084072120E-07,
 -0.330568966520E-09,
  0.650244032700E-12,
 -0.191974955040E-15,
 -0.125366004970E-17,
  0.214892175690E-20,
 -0.143880417820E-23,
  0.359608994810E-27
};

//Same as above but for converting voltages to temperatures
double TypeEVolttoTemp[] = 
{
  0.0000000E+00,
  1.7057035E+01,
 -2.3301759E-01,
  6.5435585E-03,
 -7.3562749E-05,
 -1.7896001E-06,
  8.4036165E-08,
 -1.3735879E-09,
  1.0629823E-11,
 -3.2447087E-14
};

//void setup() is only run once when the arduino is powered on or reset
void setup() 
{
  Wire.begin(); //remember that wire controls communication with i2c devices (the sensors in this case) so we start that up
  Serial.begin(115200); //this is the generic usb communication (USB = Universal Serial Bus). So we start broadcasting serial data at 115200 bits per second
  SerialBT.begin("Thermocouple_DAQ"); //Similar to above except we will be broadcasting data over bluetooth as well. "Thermocouple_DAQ" will show up when looking for bluetooth devices 
  if(!adc.init())
  {
    Serial.println("ADS1115 not connected");
  }
  if(adc.init())
  {
    Serial.println("ADS1115 Connected");  
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_0256); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  //adc.setCompareChannels(ADS1115_COMP_0_GND); //uncomment if you want to change the default

  /* Set number of conversions after which the alert pin will assert
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  adc.setConvRate(ADS1115_64_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

   /* Choose maximum limit or maximum and minimum alert limit (window) in volts - alert pin will 
   *  assert when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin assertion will be 
   *  be cleared (if not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion
   * will be cleared with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  if(tmp117.begin(0x49,Wire)==true)
  {
    Serial.println("TMP117 connected");
  }
  if(tmp117.begin(0x49,Wire)==false)
  {
    Serial.println("TMP117 not connected");
  }

  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 11);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();  
}

void loop() 
{
  unsigned long currentMillis = millis();
  
  sensors.requestTemperatures(); // Send the command to get temperatures

  float tempC = sensors.getTempC(insideThermometer);
  double ColdJunctionVoltage = Temp2Volt(tempC,TypeETemptoVolt);
  
  for (int i = 0; i < NumberOfThermocouples; i++) 
  {
    my_mux.channel(i);
    delay(2);
    adc.startSingleMeasurement();
    while(adc.isBusy()){}
    TC_Voltage[i] = adc.getResult_mV();
    //TC_Voltage[i] = adc.getRawResult();
    //Serial.println(TC_Voltage[0],0);
    //Serial.print(TC_Voltage[i],4);Serial.print(",");
    //SerialBT.print(TC_Voltage[i],4);SerialBT.print(",");
    Serial.print(Volt2Temp(ColdJunctionVoltage+TC_Voltage[i],TypeEVolttoTemp),2);Serial.print(",");
    SerialBT.print(Volt2Temp(ColdJunctionVoltage+TC_Voltage[i],TypeEVolttoTemp),2);SerialBT.print(",");
  }
    
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  
  Serial.print(tempC);Serial.print(",");
  SerialBT.print(tempC);SerialBT.print(",");

  Serial.print(millis()-currentMillis);
  SerialBT.print(millis()-currentMillis);

  Serial.println("");
  SerialBT.println("");  
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function for converting temperature into voltage for thermocouples
double Temp2Volt(double TC_Temp,double Calibration[])
{
  double TC_Volt = 0.0;
  
  for (int i = 0; i < sizeof(Calibration); ++i)
  {
    TC_Volt+=Calibration[i]*pow(TC_Temp,i);
  }
  return(TC_Volt);
}

// function for converting voltage into temperature for thermocouples
double Volt2Temp(double TC_Volt,double Calibration[])
{
  double TC_Temp = 0.0;
  
  for (int i = 0; i < sizeof(Calibration); ++i)
  {
    TC_Temp+=Calibration[i]*pow(TC_Volt,i);
  }
  return(TC_Temp);
}
