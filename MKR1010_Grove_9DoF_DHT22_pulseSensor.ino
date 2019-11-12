#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "Filter.h"
#include "DHT.h"
#include <SPI.h>
#include <WiFiNINA.h>

#define DHTPIN 2     //DHT22 PIN
#define DHTTYPE DHT22
#define SERIAL Serial
#define sample_num_mdate  5000


//PULSE SENSOR VARIABLES
int PulseSensorPurplePin = 0;
int Signal; // holds the incoming raw data. Signal value can range from 0-1024
int soglia = 802; //define a threshold for the raw Signal, use getting started example to set it

//DHT22 variables
DHT dht(DHTPIN, DHTTYPE);
float hum;
float temp;

unsigned int lastMAX = 0;

//WiFi variables
//NOTE! this was a public wifi setup without any password at all, refere to WiFiNINA example to setup a protected connection
char ssid[] = "YourSSID";
int status = WL_IDLE_STATUS;
WiFiClient client;
IPAddress server(0, 0, 0, 0); //ip address of the server local or remote
unsigned long lastConnectionTime = 0; // last time you connected to the server, in milliseconds


//---Grove 9DoF variables
const unsigned long postingInterval = 100L; // delay between updates, in milliseconds
MPU9250 accelgyro;
I2Cdev   I2C_M;
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;
float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];
static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;
volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;
volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

unsigned int ttot = 0;
unsigned int timeLast = 0;
float bpm;


ExponentialFilter<float> FilterValue(40, 0); //filter for breathing Axyz[2] axis (grove sensor)
ExponentialFilter<float> beat(40, 780); //filter for pulse sensor


void setup() {
  Serial.begin(38400);


  //----WiFi connection setup----
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid);
    delay(10000);
  }
  printWifiStatus();


  //---9DoF setup---
  Wire.begin();
  accelgyro.initialize();
  delay(1000);

  //---DHT22 setup---
  dht.begin();

  //---PULSE sensor setup---
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
}



void loop()
{

  float temp_hum_val[2] = {0};

  if (millis() - lastConnectionTime > postingInterval) { //update frequency
    float media = 0; //bpm average value
    int f = 0; //beats recorded
    unsigned int timeNow = millis();
    int i;
    int beatF; //filtred beat value
    bool beating = false;

    while (millis() - timeNow < 2000) { //average BPM in 2 seconds
      //simple way to find the peak out of the raw pulse signal
      if (beatF > soglia && !beating) {
        for (i = 0; i < 5; i++) {
          Signal = analogRead(PulseSensorPurplePin); //get the raw signal of the pulse
          beat.Filter(Signal); //filter the raw signal
          beatF = beat.Current(); //get the filtrated pulse signal
          delay(10);
          if (beatF < soglia) //false read
            break;
        }
        if (i == 5) {// 5 consecutive reading with a increase -> a beat is counted
          beating = true;

          //counting the average
          ttot = millis() - timeLast;
          timeLast = millis();
          bpm = 1000 * 60 / ttot;
          media += bpm;
          f++;
        }
        else {
          beating = false;
        }
      }
      else if (beatF <= soglia) {
        beating = false;
      }
      Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
      beat.Filter(Signal);
      beatF = beat.Current();
    }

    //calculate average
    if (f != 0)
      media = media / f;
    else {
      media = 0;
    }

    //read DHT values
    if (!dht.readTempAndHumidity(temp_hum_val)) {
      hum = temp_hum_val[0];
      temp = temp_hum_val[1];
    }

    //get grove sensor data
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();       //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    //filter out the value of the Z-ax of the Accelerometer (for breathing analysis)
    FilterValue.Filter(Axyz[2] * 1000);
    float valueResp = FilterValue.Current();


    //create the get string to be sent out to the server
    String dataGet = "gyroX=" + String(Gxyz[0], 2) + "&gyroY=" + String(Gxyz[1], 2) + "&gyroZ=" + String(Gxyz[2], 2);
    dataGet += "&compassX=" + String(Mxyz[0], 2) + "&compassY=" + String(Mxyz[1], 2) + "&compassZ=" + String(Mxyz[2], 2);
    dataGet += "&accelX=" + String(Axyz[0], 2) + "&accelY=" + String(Axyz[1], 2) + "&accelZ=" + String(valueResp, 2);
    dataGet += "&angleX=" + String(heading, 2) + "&anglePlane=" + String(tiltheading, 2);
    dataGet += "&temperature=" + String(temp, 2) + "&humidity=" + String(hum, 2) + "&bpm=" + String(media, 1);
    httpRequest(dataGet); //send the get request

  }


}


void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}

void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  //Mxyz[0] = (double) mx * 1200 / 4096;
  //Mxyz[1] = (double) my * 1200 / 4096;
  //Mxyz[2] = (double) mz * 1200 / 4096;
  Mxyz[0] = (double) mx * 4800 / 8192;
  Mxyz[1] = (double) my * 4800 / 8192;
  Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}


// this method makes a HTTP connection to the server:
void httpRequest(String jsonData) {
  // close any connection before send a new request.
  // This will free the socket on the Nina module
  client.stop();

  if (client.connect(server, 80)) {
    Serial.println("connecting...");
    // send the HTTP PUT request:
    client.println("GET /api/arduino_store_data.php?" + jsonData + " HTTP/1.1");
    client.println("Host: example.org");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();
    Serial.println(jsonData);
    Serial.println("POST1");
    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }

}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
