/******************************************************************************
  SparkFun_Photon_Weather_Basic_Soil_Meters.ino
  SparkFun Photon Weather Shield basic example with soil moisture and temp
  and weather meter readings including wind speed, wind direction and rain.
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015

  Modifications and corrections: Brandon Mutari July 13, 2016

  Based on the Wimp Weather Station sketch by: Nathan Seidle
  https://github.com/sparkfun/Wimp_Weather_Station

  This sketch prints the temperature, humidity, barometric pressure, altitude,
  soil moisture, and soil temperature to the Seril port. This sketch also
  incorporates the Weather Meters avaialbe from SparkFun (SEN-08942), which allow
  you to measure Wind Speed, Wind Direction, and Rainfall. Upload this sketch
  after attaching a soil moisture and or soil temperature sensor and Wetaher
  Meters to test your connections.

  Hardware Connections:
	This sketch was written specifically for the Photon Weather Shield,
	which connects the HTU21D and MPL3115A2 to the I2C bus by default.
  If you have an HTU21D and/or an MPL3115A2 breakout,	use the following
  hardware setup:
      HTU21D ------------- Photon
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- D1/SCL
       DA ------------------- D0/SDA

    MPL3115A2 ------------- Photon
      GND ------------------- GND
      VCC ------------------- 3.3V (VCC)
      SCL ------------------ D1/SCL
      SDA ------------------ D0/SDA

    Soil Moisture Sensor ----- Photon
        GND ------------------- GND
        VCC ------------------- D5
        SIG ------------------- A1

    DS18B20 Temp Sensor ------ Photon
        VCC (Red) ------------- 3.3V (VCC)
        GND (Black) ----------- GND
        SIG (White) ----------- D4


  Development environment specifics:
  	IDE: Particle Dev
  	Hardware Platform: Particle Photon
                       Particle Core

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.
*******************************************************************************/
#include "SparkFun_Photon_Weather_Shield_Library/SparkFun_Photon_Weather_Shield_Library.h"
#include "OneWire/OneWire.h"
#include "spark-dallas-temperature/spark-dallas-temperature.h"
//OneWire and DallasTemperature libraries are needed for DS18B20 Temp sensor
#include "math.h"   //For Dew Point Calculation

#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 11
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define SOIL_MOIST A1
#define SOIL_MOIST_POWER D5

int WDIR = A0;
int RAIN = D2;
int WSPEED = D3;

//Run I2C Scanner to get address of DS18B20(s)
//(found in the Firmware folder in the Photon Weather Shield Repo)
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/
DeviceAddress inSoilThermometer =
{0x28, 0x16, 0x81, 0xA7, 0x07, 0x00, 0x00, 0xA4};//Waterproof temp sensor address
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
long lastWindCheck = 0;
volatile float dailyrainin = 0; // [rain inches so far today in local time]

float humidity = 0;
float humTempF = 0;
float baroTempF = 0;
float tempC = 0;
float tempF = 0;
float dewptC = 0;
float dewptF = 0;
double InTempC = 0;//original temperature in C from DS18B20
float soiltempf = 0;//converted temperature in F from DS18B20
float pascals = 0;
float inches = 0;
float altitude = 0;
float hectopa = 0;
float pressure = 0;
int soilMoisture = 0;

int count = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather sensor;

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);//predeclare to compile

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

//Wunderground Vars

char SERVER[] = "rtupdate.wunderground.com"; //Rapidfire update server - for multiple sends per minute
//char SERVER [] = "weatherstation.wunderground.com"; //Standard server - for sends once per minute or less
char WEBPAGE [] = "GET /weatherstation/updateweatherstation.php?";

//Station Identification
char ID [] = "KCODENVE289"; //Your station ID here
char PASSWORD [] = "E6JVHIR2"; //your Weather Underground password here

TCPClient client;

//---------------------------------------------------------------
void setup()
{
    // DS18B20 initialization
    sensors.begin();
    sensors.setResolution(inSoilThermometer, TEMPERATURE_PRECISION);

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

    pinMode(SOIL_MOIST_POWER, OUTPUT);//power control for soil moisture
    digitalWrite(SOIL_MOIST_POWER, LOW);//Leave off by defualt

    Serial.begin(9600);   // open serial over USB

    // Make sure your Serial Terminal app is closed before powering your device
    // Now open your Serial Terminal, and hit any key to continue!
    //Serial.println("Press any key to begin");
    //This line pauses the Serial port until a key is pressed
    //while(!Serial.available()) Spark.process();

    //Initialize the I2C sensors and ping them
    sensor.begin();

    /*You can only receive acurate barrometric readings or acurate altitiude
    readings at a given time, not both at the same time. The following two lines
    tell the sensor what mode to use. You could easily write a function that
    takes a reading in one made and then switches to the other mode to grab that
    reading, resulting in data that contains both acurate altitude and barrometric
    readings. For this example, we will only be using the barometer mode. Be sure
    to only uncomment one line at a time. */
    sensor.setModeBarometer();//Set to Barometer Mode
    //sensor.setModeAltimeter();//Set to altimeter Mode

    //These are additional MPL3115A2 functions the MUST be called for the sensor to work.
    sensor.setOversampleRate(7); // Set Oversample rate
    //Call with a rate from 0 to 7. See page 33 for table of ratios.
    //Sets the over sample rate. Datasheet calls for 128 but you can set it
    //from 1 to 128 samples. The higher the oversample rate the greater
    //the time between data samples.

    sensor.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt

    seconds = 0;
    lastSecond = millis();

    // attach external interrupt pins to IRQ functions
    attachInterrupt(RAIN, rainIRQ, FALLING);
    attachInterrupt(WSPEED, wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();
}
//---------------------------------------------------------------
void loop()
{
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
  {

    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    float currentSpeed = get_wind_speed();
    //float currentSpeed = random(5); //For testing
    int currentDirection = get_wind_direction();
    windspdavg[seconds_2m] = (int)currentSpeed;
    winddiravg[seconds_2m] = currentDirection;
    //if(seconds_2m % 10 == 0) displayArrays(); //For testing

    //Check to see if this is a gust for the minute
    if(currentSpeed > windgust_10m[minutes_10m])
    {
      windgust_10m[minutes_10m] = currentSpeed;
      windgustdirection_10m[minutes_10m] = currentDirection;
    }

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph)
    {
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }

    if(++seconds > 59)
    {
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;

      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
    }

    //Get readings from all sensors
    getWeather();
    //Rather than use a delay, keeping track of a counter allows the photon to
    // still take readings and do work in between printing out data.
    count++;
    //alter this number to change the amount of time between each reading
    if(count == 10)
    {
       printInfo();
       sendToWU(); //Send data to Weather Underground
       count = 0;
    }
  }
}
//---------------------------------------------------------------
void getSoilTemp()
{
    //get temp from DS18B20
    sensors.requestTemperatures();
    update18B20Temp(inSoilThermometer, InTempC);
    //Every so often there is an error that throws a -127.00, this compensates
    if(InTempC < -100)
      soiltempf = soiltempf;//push last value so data isn't out of scope
    else
      soiltempf = (InTempC * 9)/5 + 32;//else grab the newest, good data
}
//---------------------------------------------------------------
void getSoilMositure()
{
    /*We found through testing that leaving the soil moisture sensor powered
    all the time lead to corrosion of the probes. Thus, this port breaks out
    Digital Pin D5 as the power pin for the sensor, allowing the Photon to
    power the sensor, take a reading, and then disable power on the sensor,
    giving the sensor a longer lifespan.*/
    digitalWrite(SOIL_MOIST_POWER, HIGH);
    delay(200);
    soilMoisture = (analogRead(SOIL_MOIST)/3500)*100;//choose a top end for 100% saturation
    delay(100);
    digitalWrite(SOIL_MOIST_POWER, LOW);

}
//---------------------------------------------------------------
void update18B20Temp(DeviceAddress deviceAddress, double &tempC)
{
  tempC = sensors.getTempC(deviceAddress);
}
//---------------------------------------------------------------
//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int analogRaw;

  analogRaw = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  //Wind Vains may vary in the values they return. To get exact wind direction,
  //it is recomended that you AnalogRead the Wind Vain to make sure the values
  //your wind vain output fall within the values listed below.
  if(analogRaw >= 2200 && analogRaw < 2400) return (270);//W
  if(analogRaw >= 2100 && analogRaw < 2200) return (293);//WNW
  if(analogRaw >= 3200 && analogRaw < 3299) return (315);//NW
  if(analogRaw >= 3100 && analogRaw < 3200) return (337);//NNW
  if(analogRaw >= 3890 && analogRaw < 3999) return (0);//N
  if(analogRaw >= 3700 && analogRaw < 3780) return (23);//NNE
  if(analogRaw >= 3780 && analogRaw < 3890) return (45);//NE
  if(analogRaw >= 3400 && analogRaw < 3500) return (67);//ENE
  if(analogRaw >= 3570 && analogRaw < 3700) return (90);//E
  if(analogRaw >= 2600 && analogRaw < 2700) return (113);//ESE
  if(analogRaw >= 2750 && analogRaw < 2850) return (135);//SE
  if(analogRaw >= 1510 && analogRaw < 1580) return (157);//SSE
  if(analogRaw >= 1580 && analogRaw < 1650) return (180);//S
  if(analogRaw >= 1470 && analogRaw < 1510) return (203);//SSW
  if(analogRaw >= 1900 && analogRaw < 2000) return (225);//SW
  if(analogRaw >= 1700 && analogRaw < 1750) return (247);//WSW
  if(analogRaw > 4000) return(-1); // Open circuit?  Probably means the sensor is not connected
  return -1;
}
//---------------------------------------------------------------
//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  /* Serial.println();
   Serial.print("Windspeed:");
   Serial.println(windSpeed);*/

  return(windSpeed);
}
//---------------------------------------------------------------
//Returns the altitude corrected pressure for a given elevation
//equation found here: http://www.srh.noaa.gov/images/epz/wxcalc/altimeterSetting.pdf
float getAltPressure()
{
  altitude = 1655.978; //altitude in meters
  hectopa = sensor.readPressure() / 100;
  pressure = (hectopa - 0.3) * pow((1 + ((pow(1013.25,0.190284) * 0.0065)/288) * (altitude/(pow((hectopa - 0.3),0.190284)))),(1/0.190284));
  inches = pressure * 0.02953;

  return(inches);
}
//---------------------------------------------------------------
void getWeather()
{
    // Measure Relative Humidity from the HTU21D or Si7021
    humidity = sensor.getRH();

    // Measure Temperature from the HTU21D or Si7021
    humTempF = sensor.getTempF(); //temp correction applied
    // Temperature is measured every time RH is requested.
    // It is faster, therefore, to read it from previous RH
    // measurement with getTemp() instead with readTemp()

    //Measure the Barometer temperature in F from the MPL3115A2
    baroTempF = sensor.readBaroTempF();

    //Average temperature readings between the humidity and pressure sensor
    tempF = ((humTempF + baroTempF)/2)-4;//temp correction applied
    tempC = (5*(tempF-32))/9;

    //Measure Pressure from the MPL3115A2
    getAltPressure();

    //If in altitude mode, you can get a reading in feet  with this line:
    //float altf = sensor.readAltitudeFt();

    getSoilTemp();//Read the DS18B20 waterproof temp sensor
    getSoilMositure();//Read the soil moisture sensor

    //Calc winddir
    winddir = get_wind_direction();

    //Calc windspeed
    windspeedmph = get_wind_speed();

    //Calc windspdmph_avg2m
    float temp = 0;
    for(int i = 0 ; i < 120 ; i++)
      temp += windspdavg[i];
    temp /= 120.0;
    windspdmph_avg2m = temp;

    //Calc winddir_avg2m
    temp = 0; //Can't use winddir_avg2m because it's an int
    for(int i = 0 ; i < 120 ; i++)
      temp += winddiravg[i];
    temp /= 120;
    winddir_avg2m = temp;

    //Calc windgustmph_10m
    //Calc windgustdir_10m
    //Find the largest windgust in the last 10 minutes
    for(int i = 0; i < 10 ; i++)
    {
      if(windgust_10m[i] > windgustmph_10m)
      {
        windgustmph_10m = windgust_10m[i];
        windgustdir_10m = windgustdirection_10m[i];
      }
    }

    //Total rainfall for the day is calculated within the interrupt
    //Calculate amount of rainfall for the last 60 minutes
    rainin = 0;
    for(int i = 0 ; i < 60 ; i++)
      rainin += rainHour[i];

    //Calculate Dew Point
    dewptC = dewPoint(tempC, humidity);
    dewptF = (dewptC * 9.0)/ 5.0 + 32.0;
}
//---------------------------------------------------------------
// dewPoint function from NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//---------------------------------------------------------------
double dewPoint(double celsius, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}
//---------------------------------------------------------------
void printInfo()
{
  //This function prints the weather data out to the default Serial Port
  Serial.println("Data to send ----------------");

  Serial.print("tempf=");
  Serial.print(tempF);
  Serial.print(",");

  Serial.print("dewptf=");
  Serial.print(dewptF);
  Serial.print(",");

  Serial.print("hum=");
  Serial.print(humidity);
  Serial.print(",");

  Serial.print("baromin=");
  Serial.print(inches);
  Serial.println(",");

  Serial.print("winddir=");
  Serial.print(winddir);
  Serial.print(",");

  Serial.print("windspeedmph=");
  Serial.print(windspeedmph);
  Serial.print(",");

  Serial.print("windgustmph=");
  Serial.print(windgustmph);
  Serial.print(",");

  Serial.print("windgustdir=");
  Serial.print(windgustdir);
  Serial.print(",");

  Serial.print("windspdmph_avg2m=");
  Serial.print(windspdmph_avg2m);
  Serial.print(",");

  Serial.print("winddir_avg2m=");
  Serial.print(winddir_avg2m);
  Serial.print(",");

  Serial.print("windgustmph_10m=");
  Serial.print(windgustmph_10m);
  Serial.print(",");

  Serial.print("windgustdir_10m=");
  Serial.print(windgustdir_10m);
  Serial.println(",");

  Serial.print("rainin=");
  Serial.print(rainin);
  Serial.print(",");

  Serial.print("dailyrainin=");
  Serial.print(dailyrainin);
  Serial.println(",");

  Serial.print("soiltempf=");
  Serial.print(soiltempf);
  Serial.print(",");

  Serial.print("soilmoisture=");
  Serial.print(soilMoisture);

  Serial.println("");
}
//---------------------------------------------------------------
void sendToWU()
{
  if (client.connect(SERVER, 80)) {
  client.print(WEBPAGE);
  client.print("ID=");
  client.print(ID);

  client.print("&PASSWORD=");
  client.print(PASSWORD);

  client.print("&dateutc=now");//can use 'now' instead of time if sending in real time

  client.print("&tempf=");
  client.print(tempF);

  client.print("&dewptf=");
  client.print(dewptF);

  client.print("&humidity=");
  client.print(humidity);

  client.print("&baromin=");
  client.print(inches);

  client.print("&winddir=");
  client.print(winddir);

  client.print("&windspeedmph=");
  client.print(windspeedmph);

  client.print("&windgustmph=");
  client.print(windgustmph);

  client.print("&windgustdir=");
  client.print(windgustdir);

  client.print("&windspdmph_avg2m=");
  client.print(windspdmph_avg2m);

  client.print("&winddir_avg2m=");
  client.print(winddir_avg2m);

  client.print("&windgustmph_10m=");
  client.print(windgustmph_10m);

  client.print("&windgustdir_10m=");
  client.print(windgustdir_10m);

  client.print("&rainin=");
  client.print(rainin);

  client.print("&dailyrainin=");
  client.print(dailyrainin);

  client.print("&soiltempf=");
  client.print(soiltempf);

  client.print("&soilmoisture=");
  client.print(soilMoisture);

  //client.print("&action=updateraw");    //Standard update rate - for sending once a minute or less
  client.print("&softwaretype=Particle-Photon&action=updateraw&realtime=1&rtfreq=5");  //Rapid Fire update rate - for sending multiple times per minute, specify frequency in seconds
  client.println();
  Serial.println("WU upload complete");
  Serial.println();
  delay(300);//Without the delay it goes to sleep too fast and the send is unreliable
  }else{
    Serial.println(F("WU connection failed"));
  return;
  }
}
