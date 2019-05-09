//Integrated code for the SKEPI project - Gymnasio Antirriou
// 2-5-2019
// V 2.0

#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <string.h>
#include <NMEAGPS.h>

// **************** GENERIC DEFS - START ***************************

#define PAUSE_TIME 4000 // The time, in milliseconds, between successive posts to the IoT platform

// **************** GENERIC DEFS - END ***************************



// **************** FFT DEFS - START ***************************
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128 ; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
// **************** FFT DEFS - END ***************************

// ***

// **************** AES DEFS - START ***************************
#include <AES.h>
#include "./printf.h"
#define AES_PLAINTEXT_SIZE 256

AES aes ;

byte *key = (unsigned char*) "0123456789010123"; // 128-bit key

byte cipher_to_send[AES_PLAINTEXT_SIZE] ;
byte plain[AES_PLAINTEXT_SIZE] ; // = "Add NodeAdd NodeAdd NodeAdd NodeAdd Node";
int plainLength = sizeof(plain)-1;  // don't count the trailing /0 of the string !
int padedLength = plainLength + N_BLOCK - plainLength % N_BLOCK;

// real iv = iv x2 ex: 01234567 = 0123456701234567
unsigned long long int my_iv = 36753562;

// **************** AES DEFS - END ***************************

// ***

// **************** GPS DEFS - START ***************************
// For the function that test whether a given point is inside a polygon
#define R_earth 6371 // Approcimate Earth's radius in Km
#define POLYGON_VERTICES 4 // The number of points defining the allowed geographical area
#define OUTSIDE 0
#define INSIDE 1
#define INF 10000
#define gps_port Serial1 // TX/RX port
#define MAX_STRLEN 80

// Macro definitions.
#define MAX(x,y) (x >= y ? x : y)
#define MIN(x,y) (x >= y ? y : x)

byte nCount ;
char sString[MAX_STRLEN+1];

NMEAGPS gps;
gps_fix fix;


struct point // As defined by Google Maps.
  {
   double Latitude, Longitude ;
   double x, y ;
  } ;

struct point allowed_polygon[POLYGON_VERTICES] =
  { /* Google Map coordinates. */
   {.Latitude = 38.328597, .Longitude = 21.763881, .x = 0, .y = 0},
   {.Latitude = 38.329614, .Longitude = 21.768038, .x = 0, .y = 0},
   {.Latitude = 38.331902, .Longitude = 21.759302, .x = 0, .y = 0},
   {.Latitude = 38.336043, .Longitude = 21.764067, .x = 0, .y = 0}
  } ;

struct point allowed_polygon_only_x_y[POLYGON_VERTICES] ;

point p ; // The current position (Cartesian coordinates) of the person.

void convert_spherical_to_Cartesian(double Lat, double Lng, double &x_coord, double &y_coord) ;
void set_polygon_coordinates_Cartesian(struct point *allowed_p, int num_of_points) ;

// **************** GPS DEFS - END ***************************

// ***

// *********************** ACCELERATOR/GYROSCOPE DEFS - START ***********
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// *********************** ACCELERATOR/GYROSCOPE DEFS - END ***********

// ***

// *********************** WiFi CONNECTION DEFS - START ***********
const char WIFI_SSID[] = "OTE7A3E09"; // WiFI ssid 
const char WIFI_PASS[] = "0437342360924320"; //WiFI password

// const char WIFI_SSID[] = "cti-wireless"; // WiFI ssid 
// const char WIFI_PASS[] = "";

// const char WIFI_SSID[] = "VodafoneMobileWiFi-24DE74"; // WiFI ssid 
// const char WIFI_PASS[] = "4998241413"; //WiFI password
// *********************** WiFi CONNECTION DEFS - END ***********

// ***

// *********************** WIA-IOT-PLATFORM DEFS - START ***********
// get this from the wia dashboard. it should start with `d_sk`
const char* device_secret_key = "d_sk_EgTaOFJuvi6bi89jFEhV4N7z";

WiFiClient client;
int status = WL_IDLE_STATUS;

// Wia API parameters
char server[] = "api.wia.io";
char path[] = "/v1/events";
int port = 80;

HttpClient httpClient = HttpClient(client, server, port);
// StaticJsonDocument<200> jsonBuffer;
// JsonObject root = jsonBuffer.to<JsonObject>();


// *********************** WIA-IOT-PLATFORM DEFS - END ***********

// 
// Regards Serial OutPut  -- Set This Up to your needs
static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse 
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog from A5. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduino finds a beat.

//

void setup() {
  delay(2000) ;
  Serial.println("Ready!");
  pinMode(0, INPUT); // ECG - Setup for leads off detection LO +
  pinMode(1, INPUT); // ECG - Setup for leads off detection LO -

  // initialize serial communications and wait for port to open:
  Serial.begin(9600);
  gps_port.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting WiFI connection to Wia!");
 
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    // printWifiStatus();
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    printWifiStatus();
    
    Serial.print("Status: ");
    Serial.println(String(status));
    // wait 2 seconds for connection:
    // delay(2000);
  }


  // ********************** ACCELERATOR/GYROSCOPE SETUP ********
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop()
{

 StaticJsonDocument<200> jsonBuffer;
 JsonObject root = jsonBuffer.to<JsonObject>();

 root["name"] = "Warming up!";
 root["data"] = "GYMNASIO ANTIRRIOU!";
 int val1, val2 ; // ECG related variables
 long start, finish ; // ECG related variables
 // byte outputblob[100] ;
 
 // GPS
 int inside = -1, i ; // GPS related variables

 // ******* GPS ******
 while (gps.available( gps_port ))
   {
    fix = gps.read();
    Serial.println("GPS - connected.") ;
    // Make sure we have a location to send
    if (fix.valid.location)
      {
//     Serial.print( F("https://maps.google.com/maps/place/") );
//     Serial.print( fix.latitude(), 5 );
//     Serial.print( F(", ") );
//     Serial.println( fix.longitude(), 5 );

       p.Latitude = fix.latitude() ;
       p.Longitude = fix.longitude() ;
//     Serial.print(F("\n")) ;
//     Serial.print(p.x,6);
//     Serial.print(F("\n")) ;
//     Serial.print(p.y,6) ;
     
//       p.Latitude = 38.327195 ;
//       p.Latitude = 21.765826 ;
//     Serial.print(F("\n")) ;
//     Serial.print(p.x,6);
     
//     Serial.print(F("\n")) ;
//     Serial.print(p.y,6);

       convert_spherical_to_Cartesian(p.Latitude,p.Longitude,&p.x, &p.y);
       set_polygon_coordinates_Cartesian(allowed_polygon, POLYGON_VERTICES) ;

       for (i = 0 ; i < POLYGON_VERTICES ; i++)
          {
           allowed_polygon_only_x_y[i].x = allowed_polygon[i].x ;
           allowed_polygon_only_x_y[i].y = allowed_polygon[i].y ;
          }       
//      inside = InsidePolygon(allowed_polygon, POLYGON_VERTICES, p) ;

        inside = isInside(allowed_polygon_only_x_y, POLYGON_VERTICES, p) ;
    
 /*       if (inside == INSIDE)
          Serial.println("Inside!!!") ;
        else
          Serial.println("Outside, alert!!!") ;
          */
     }
   } // "while" GPS

 //************* ACCELERATOR/GYROSCOPE *********
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
// Serial.print("AcX = "); Serial.print(AcX);
// Serial.print(" | AcY = "); Serial.print(AcY);
// Serial.print(" | AcZ = "); Serial.print(AcZ);
// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
// Serial.print(" | GyX = "); Serial.print(GyX);
// Serial.print(" | GyY = "); Serial.print(GyY);
// Serial.print(" | GyZ = "); Serial.println(GyZ);
 
 // delay(333);


 // Start taking ECG raw data 
 
 Serial.println("Starting taking ECG samples ...");
 double cycles = (((samples-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
 for (uint16_t i = 0; i < samples; i++)
   {
    int readingIn = analogRead(A5) ;
    ISR(readingIn) ; // Take current ECG sample into account for estimating the heartbeat rate
    vReal[i] = (double ) readingIn ; // Read ECG data
    // vReal[i] = int8_t((amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0);/* Build data with positive and negative values*/
    // vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
   }
  /* Print the results of the simulated sampling according to time */

// Serial.println("Sampled data follow below:");
// PrintVector(vReal, samples, SCL_TIME);
 FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
// Serial.println("Weighed data:");
// PrintVector(vReal, samples, SCL_TIME);
 FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
// Serial.println("Computed Real values:");
// PrintVector(vReal, samples, SCL_INDEX);
// Serial.println("Computed Imaginary values:");
// PrintVector(vImag, samples, SCL_INDEX);
 FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
// Serial.println("Computed magnitudes:");
// PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
 double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
 
 Serial.print("Frequency with maximum FFT amplitude: ") ;
 Serial.println(x, 6);
 Serial.print("BPM is now: ") ;
 Serial.println(BPM) ;

//while(1); /* Run Once */
// delay(2000); /* Continue after short delay */
  
 
 // Now we transfer the events from Accelerometer/Gyroscope, GPS, and ECG to the Wia platform
 // At the platform, triggers by the events will notify, as needed, the designated alert recipients
  if (client.connect(server, port))
   {
    char outputblob[AES_PLAINTEXT_SIZE]  ;
    byte cipher_to_send[AES_PLAINTEXT_SIZE] ;

//    char cipheroutput[AES_PLAINTEXT_SIZE * 3+1] ;
//    cipheroutput[0] = '\0' ;
    

    for (i = 0 ; i < AES_PLAINTEXT_SIZE ; i++)
        outputblob[i] = ' ' ;
  
    sprintf(outputblob, "Lat: %f, Long: %f, Tmp: %f, BPM: %f, AcX: %d, AcY: %d, AcZ (Fall): %d, ECG Maj. Peak.: %f", p.Latitude, p.Longitude, (double ) Tmp/340.00+36.53, (double ) BPM, AcX, AcY, AcZ, x) ;

    if (inside == INSIDE)
      root["name"] = "INSIDE" ;
    else
      if (inside == OUTSIDE)
        root["name"] = "OUTSIDE" ;

    if (AcZ < 15700 - 3000 || AcZ > 15700 + 3000)
      if (root["name"] == "OUTSIDE")
        root["name"] = "OUTSIDE+FALL" ;
      else
        if (root["name"] == INSIDE)
          root["name"] = "INSIDE+FALL" ;
        else
          root["name"] = "FALL" ;
    
   if ((double ) Tmp/340.00+36.53 > 40 || (double ) Tmp/340.00+36.53 < 10)
     root["name"] = "TMP" ; // low/high temperature problem - Overrides previous - it is critical

   if (BPM > 80 || BPM < 50)
     root["name"] = "BPM" ; // Cardiac puld problem - Overrides previous - it is critical
        
   for (i = 0 ; i < AES_PLAINTEXT_SIZE ; i++)
       plain[i] = outputblob[i] ;

    // Encrypt data
    prekey(128) ;

    root["data"] = outputblob ;
    postToWia(root);
   }
 else
   {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
   }
 delay(PAUSE_TIME); // Wait for 4 seconds to post again
}

void postToWia(JsonObject& data){
    String dataStr = ""; 
    serializeJson(data, dataStr); 
    httpClient.beginRequest();
    httpClient.post(path);
    httpClient.sendHeader("Content-Type", "application/json");
    httpClient.sendHeader("Content-Length", dataStr.length());
    httpClient.sendHeader("Authorization", "Bearer "  +    String(device_secret_key));
    httpClient.beginBody();
    httpClient.print(dataStr);
    httpClient.endRequest();

    client.flush(); 
}

void printWifiStatus() {
 // print the SSID of the network you're attached to:
 delay(2000) ;
 Serial.print("SSID: ");
 Serial.println(WiFi.SSID());

 // print your WiFi shield's IP address:
 IPAddress ip = WiFi.localIP();
 Serial.print("IP Address: ");
 Serial.println(ip);

 // print the received signal strength:
 long rssi = WiFi.RSSI();
 Serial.print("signal strength (RSSI):");
 Serial.print(rssi);
 Serial.println(" dBm");
}

// GPS functions follow
void convert_spherical_to_Cartesian(double Lat, double Lng, double *x_coord, double *y_coord)
{
 double Latrad, Lngrad ;
 
// Convert to radians
 Latrad = (Lat * PI) / 180 ;
 Lngrad = (Lng * PI) / 180;

// Convert spherical (i.e. Latitude/Longiture) to Cartesian coordinates
 *x_coord = R_earth * cos(Latrad)  * cos(Lngrad) ;
 *y_coord = R_earth * cos(Latrad) * sin(Lngrad) ;
}

void set_polygon_coordinates_Cartesian(struct point *allowed_p, int num_of_points)
{
 int i ;
 double x, y ;
 
 for (i = 0 ; i < num_of_points ; i++)
    {
     convert_spherical_to_Cartesian(allowed_p[i].Latitude, allowed_p[i].Longitude, &x, &y) ;
     allowed_polygon[i].x = x ;
     allowed_polygon[i].y = y ;
   
    }
}

int InsidePolygon(struct point *polygon, int N, struct point p)
{
  int i, counter = 0 ;
  double xinters ;
  point p1, p2 ;
  
  p1 = polygon[0] ;
  for (i = 1 ; i <=N ; i++) {
    p2 = polygon[i % N];
    if (p.y > MIN(p1.y, p2.y)) {
      if (p.y <= MAX(p1.y, p2.y)) {
        if (p.x <= MAX(p1.x, p2.x)) {
          if (p1.y != p2.y) {
            xinters = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x ;
            if (p1.x == p2.x || p.x <= xinters)
              counter++ ;
          }
        }
      }
    }
    p1 = p2 ;
  }
  Serial.print(F("Counter is")) ;
  Serial.print(counter) ;
  if (counter % 2 == 0)
    return(INSIDE) ;
  else
    return(OUTSIDE) ;
}

// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(point p, point q, point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 
  
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(point p, point q, point r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(point p1, point q1, point p2, point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 
  
// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(point polygon[], int n, point p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinite 
    point extreme = {INF, p.y}; 
  
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 
  
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
               return onSegment(polygon[i], p, polygon[next]); 
  
            count++; 
        } 
        i = next; 
    } while (i != 0);

    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
}

void prekey (int bits)
{
  int i ;
  aes.iv_inc();
  byte iv [N_BLOCK] ;
  byte plain_p[padedLength];
  byte cipher [padedLength] ;
  byte check [padedLength] ;
  unsigned long ms = micros ();
  aes.set_IV(my_iv);
  aes.get_IV(iv);


  aes.do_aes_encrypt(plain,plainLength,cipher,key,bits,iv);
  Serial.print("Encryption took: ");
  Serial.println(micros() - ms);
  ms = micros ();
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  aes.do_aes_decrypt(cipher,padedLength,check,key,bits,iv);

  for (i = 0 ; i < AES_PLAINTEXT_SIZE ; i++)
    { Serial.print((int ) cipher[i]) ;
     Serial.write(", ") ;
     cipher_to_send[i] = (int ) cipher[i] ;
    } 

  Serial.print("Decryption took: ");
  Serial.println(micros() - ms);
  printf("\n\nPLAIN :");
  aes.printArray(plain,(bool)true);
  printf("\nCIPHER:");
  aes.printArray(cipher,(bool)false);
  printf("\nCHECK :");
  aes.printArray(check,(bool)true);
  printf("\nIV    :");
  aes.printArray(iv,16);
  printf("\n============================================================\n");
}

void prekey_test ()
{
  prekey (128) ;
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void ISR(int anala){                       // triggered when Timer2 counts to 124
  //cli();                                      // disable interrupts while we do this
  Signal = anala;//analogRead(pulsePin);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){      // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                         // T is the trough
      T = Signal;                            // keep track of lowest point in pulse wave 
    }
  }

  if(Signal > thresh && Signal > P){        // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                         // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when there is a pulse
      IBI = sampleCounter - lastBeatTime;         // time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realistic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time beat is found
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
       /* sei();                               // enable interrupts again
        return; */                             // IBI value is unreliable so discard it
      }   
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    //digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }

 // sei();     
 // enable interrupts when youre done!
}
