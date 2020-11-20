/*Adafruit_GPS Library enables live GPS tracking 
via the Adafruit Ultimate GPS*/
#include <Adafruit_GPS.h>

/*SoftwareSerial Library enasbles the Adafruit Ultimate GPS
to communicate with the Arduino Mega microcontroller*/
#include <SoftwareSerial.h>

/*Servo Library is used for several quadcopter functionalities:
--Control four ESCs
--Manipulate continous rotation servo for the camera*/
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>

#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

/*Define the communication pins (TX andc RX) for the GPS*/
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean);

/*Five Ultrasonic distance sensors each use two pins to operate:
--"trigPin" is used to emit the ultrsonic signal from the sensor
--"echoPin" is used to receive the ultrasonic signal if it bounces off an object
--each sensor has an operating distance of 100cm
--four sensors will be used for close proximity object detection while in flight
--one sensor will be used to detect the distance from the landing pad*/
#define trigPinF 22
#define echoPinF 23

#define trigPinB 24
#define echoPinB 25

#define trigPinL 26
#define echoPinL 27

#define trigPinR 28
#define echoPinR 29

#define trigPinA 30
#define echoPinA 31

//declare camera servo
Servo cameraArticulation;

//declare ESC (motor) servos
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

//GAS sensor output pin to Arduino analog pins
const int gas2 = A0;
const int gas5 = A1;
const int gas7 = A2;
const int gas8 = A3;
const int gas135 = A4;

//motor speeds
int FL_speed = 120;
int FR_speed = 120;
int BL_speed = 120;
int BR_speed = 120;
int hover_speed = 120;

int acc_dec_rate = 10;

int posScale = 4;
int negScale = -4;

int x_center = 0;
int y_center = 0;

int compass_angle = 0;
int compass_180;
float compass_diff;
float compScale = 0.0555;

float feetConversion= 3.28084;
int cruisingAltitude = 200;
float radius_of_earth = 6378.1; // km

int obj_dist = 100;

int recon_alt = 50;
int recon_alt_buffer = 3;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void ultrasonic_Altitude() {
  long durationA, distanceA;
  digitalWrite(trigPinA, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinA, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinA, LOW);
  durationA = pulseIn(echoPinA, HIGH);
  distanceA = (durationA/2) / 29.1;
}

  void forward(){
    x_center;
    y_center;
    motorFL.write(FL_speed);
    motorFR.write(FR_speed);
    motorBL.write(BL_speed + acc_dec_rate);
    motorBL.write(BR_speed + acc_dec_rate);
  }
  void reverse(){
    x_center;
    y_center;
    motorFL.write(FL_speed + acc_dec_rate);
    motorFR.write(FR_speed + acc_dec_rate);
    motorBL.write(BL_speed);
    motorBR.write(BR_speed);
  }
  void hover(){
    x_center = 0;
    y_center = 0;
    motorFL.write(FL_speed);
    motorFR.write(FR_speed);
    motorBL.write(BL_speed);
    motorBR.write(BR_speed);
  }
  void left(){
    x_center;
    y_center;
    motorFL.write(FL_speed);
    motorFR.write(FR_speed + acc_dec_rate);
    motorBL.write(BL_speed);
    motorBR.write(BR_speed + acc_dec_rate);
  }
  void right(){
    x_center;
    y_center;
    motorFL.write(FL_speed + acc_dec_rate);
    motorFR.write(FR_speed);
    motorBL.write(BL_speed + acc_dec_rate);
    motorBR.write(BR_speed);
  }

  void leftRotate() {
    x_center = 0;
    y_center = 0;
    motorFL.write(FL_speed + acc_dec_rate);
    motorFR.write(FR_speed);
    motorBL.write(BL_speed);
    motorBR.write(BR_speed + acc_dec_rate);
  }

  void rightRotate() {
    x_center = 0;
    y_center = 0;
    motorFL.write(FL_speed);
    motorFR.write(FR_speed + acc_dec_rate);
    motorBL.write(BL_speed + acc_dec_rate);
    motorBR.write(BR_speed);  
  }

  void ascend(){
    x_center = 0;
    y_center = 0;
    motorFL.write(FL_speed + acc_dec_rate);
    motorFR.write(FR_speed + acc_dec_rate);
    motorBL.write(BL_speed + acc_dec_rate);
    motorBR.write(BR_speed + acc_dec_rate);
  }

  void descend(){
    x_center = 0;
    y_center = 0;
    motorFL.write(FL_speed - acc_dec_rate);
    motorFR.write(FR_speed - acc_dec_rate);
    motorBL.write(BL_speed - acc_dec_rate);
    motorBR.write(BR_speed - acc_dec_rate); 
  }
  
void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(trigPinF,OUTPUT);
  pinMode(echoPinF, INPUT);

  pinMode(trigPinB,OUTPUT);
  pinMode(echoPinB, INPUT);

  pinMode(trigPinL,OUTPUT);
  pinMode(echoPinL, INPUT);

  pinMode(trigPinR,OUTPUT);
  pinMode(echoPinR, INPUT);

  pinMode(trigPinA,OUTPUT);
  pinMode(echoPinA, INPUT);

  //attach camera servo to digital pin 5
  cameraArticulation.attach(5);

  //attach the ESCs (motors) to PWM digital pins
  motorFL.attach(9);
  motorFR.attach(10);
  motorBL.attach(11);
  motorBR.attach(12);

  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
    Serial.begin(9600);

  /* Initialise the sensor */
    if (!accel.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      Serial.println("Accelerometer: Disconnected");
      while (1);
    } else {
      Serial.println("Accelerometer: Connected");
    }

    if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Magnometer: Disconnected");
    while(1);
  } else {
    Serial.println("Magnometer: Connected");
  }

  //initialize ESCs
  delay(1000);
  motorFL.write(0);
  motorFR.write(0);
  motorBL.write(0);
  motorBR.write(0);
  delay(2000);
  motorFL.write(180);
  motorFR.write(180);
  motorBL.write(180);
  motorBR.write(180);
  delay(2000);
  motorFL.write(90);
  motorFR.write(90);
  motorBL.write(90);
  motorBR.write(90);
  delay(2000);


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //parse data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() 
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

double distance_in_ft(float start_lat, float start_long, float end_lat, float end_long) {
  // / 180 / PI converts degrees to radians
  start_lat/= 180 / PI; 
  start_long/= 180 / PI;
  end_lat/= 180 / PI;
  end_long/= 180 / PI;
  float a = pow(sin((end_lat-start_lat)/2), 2) + cos(start_lat) * cos(end_lat) * pow(sin((end_long-start_long)/2), 2);
  float distance = radius_of_earth * 2 * atan2(sqrt(a), sqrt(1-a));
  return double((distance/1000)/feetConversion);
}

double lat_lon_bearing(float start_lat, float start_long, float end_lat, float end_long) {
  //calculate bearing
  double y = sin(end_long-start_long) * cos(end_lat);
  double x = cos(start_lat) * sin(end_lat) - sin(start_lat) * cos(end_lat) * cos(end_long-start_long);
  return double(((atan2(y, x))*4068)/71);
}

void flight_square() {
  //user will pull out a rectangle on the app 
  //the length and width of rectangle will be used to determine how far the drone flies in each direction
  //the firefighter will also determine monitoring altitude
}

void loop() {
  if((fl_speed = 0) && (br_speed = 0)) {
    
  }
  else {
  //transmit gas sensor data via wireless connection
  Serial.print("Smoke: ");Serial.println(analogRead(gas2));
  Serial.print("Methane: ");Serial.println(analogRead(gas5));
  Serial.print("CO: ");Serial.println(analogRead(gas7));
  Serial.print("Hydrogen: ");Serial.println(analogRead(gas8));
  Serial.print("CO2: ");Serial.println(analogRead(gas135));
    /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  int lat = GPS.latitudeDegrees;
  int lon = GPS.longitudeDegrees;
  int alt = GPS.altitude;
  int satFix = GPS.fix;
  int ang = GPS.angle;

  //float latInput = Serial.read(fireLat);
  //float lonInput = Serial.read(fireLon);
  //float latStation = Serial.read(homeLat);
  //float LonStation = Serial.read(homeLon);
  int latInput = 0;
  int lonInput = 0;
  int latStation = 1;
  int lonStation = 1;
  float currentCoord[2] = {lat,lon};
  float fireCoord[2] = {latInput,lonInput};
  float stationCoord[2] = {latStation,lonStation};

  mag.getEvent(&event);
  
  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  if(compass_angle >= 180) {
    compass_180 = abs((compass_angle + 180) - 360);
  } else {
    compass_180 = compass_angle + 180;
  }

  if((compass_angle == 0) && (heading > compass_180)) {
    compass_diff = (360 - heading) * compScale;
  }
  else if((compass_angle == 0) && (heading <= compass_180)) {
    compass_diff = heading * compScale;
  }
  else if((compass_angle == 180) && (heading < compass_angle)) {
    compass_diff = (compass_angle - heading) * compScale;
  }
  else if((compass_angle == 180) && (heading >= compass_angle)) {
    compass_diff = (heading - compass_angle) * compScale;
  }
  else if((compass_angle > 180) && (heading < compass_angle)) {
    compass_diff = (heading - compass_angle) * compScale;
  }
  else if((compass_angle > 180) && (heading <= compass_180)) {
    compass_diff = (360 - (compass_angle - heading)) * compScale;
  }
  else if((compass_angle > 180) && (heading < compass_angle) && (heading > compass_180)) {
    compass_diff = (compass_angle - heading) * compScale;
  }
  else if((compass_angle < 180) && (heading < compass_angle)) {
    compass_diff = (compass_angle - heading) * compScale;
  }
  else if((compass_angle < 180) && (heading >= compass_180)) {
    compass_diff = ((360 - heading) + compass_angle) * compScale;
  }
  else if((compass_angle < 180) && (heading > compass_angle) && (heading < compass_180)) {
    compass_diff = (heading - compass_angle) * compScale;
  } else {}

     //Display the results (acceleration is measured in m/s^2)
    //Serial.print("Ang: "); Serial.print(heading); Serial.print("  ");
    //Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    //Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    //Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  accel.getEvent(&event);
    
  motorFL.write(FL_speed);
  motorFR.write(FR_speed);
  motorBL.write(BL_speed);
  motorBR.write(BR_speed);

  //flight controller stabilization with accelerometer and magnometer

  if ((event.acceleration.x > x_center) && (event.acceleration.y > y_center) && (event.acceleration.x > event.acceleration.y)) {
    BL_speed = hover_speed + (event.acceleration.x * posScale);
    BR_speed = hover_speed + (event.acceleration.x * posScale);
  }
  else if ((event.acceleration.x > x_center) && (event.acceleration.y > y_center) && (event.acceleration.y > event.acceleration.x)) {
    FR_speed = hover_speed + (event.acceleration.y * posScale);
    BR_speed = hover_speed + (event.acceleration.y * posScale);
  }
  else if ((event.acceleration.x > x_center) && (event.acceleration.y < y_center) && (event.acceleration.x > (event.acceleration.y * -1))) {
    BL_speed = hover_speed + (event.acceleration.x * posScale);
    BR_speed = hover_speed + (event.acceleration.x * posScale);
  }
  else if ((event.acceleration.x > x_center) && (event.acceleration.y < y_center) && ((event.acceleration.y * -1) > event.acceleration.x)) {
    FL_speed = hover_speed + (event.acceleration.y * negScale);
    BL_speed = hover_speed + (event.acceleration.y * negScale);
  }
  else if ((event.acceleration.x < x_center) && (event.acceleration.y > y_center) && ((event.acceleration.x * -1) > event.acceleration.y)) {
    FL_speed = hover_speed + (event.acceleration.x * negScale);
    FR_speed = hover_speed + (event.acceleration.x * negScale);
  }
  else if ((event.acceleration.x < x_center) && (event.acceleration.y > y_center) && (event.acceleration.y > (event.acceleration.x * -1))) {
    FR_speed = hover_speed + (event.acceleration.y * posScale);
    BR_speed = hover_speed + (event.acceleration.y * posScale);
  }
  else if ((event.acceleration.x < x_center) && (event.acceleration.y < y_center) && (event.acceleration.x < event.acceleration.y)) {
    FL_speed = hover_speed + (event.acceleration.x * negScale);
    FR_speed = hover_speed + (event.acceleration.x * negScale);
  }
  else if ((event.acceleration.x < x_center) && (event.acceleration.y < y_center) && (event.acceleration.y < event.acceleration.x)) {
    FL_speed = hover_speed + (event.acceleration.y * negScale);
    BL_speed = hover_speed + (event.acceleration.y * negScale);
  }
  else if (((heading) > compass_angle) && (heading < compass_180) && ((compass_diff > event.acceleration.x) || (compass_diff > event.acceleration.y) || (compass_diff > event.acceleration.x * -1) || (compass_diff > event.acceleration.y * -1))) {
    FL_speed = hover_speed + 10;
    BR_speed = hover_speed + 10;
  }
  else if (((heading) < compass_angle) && (heading > compass_180) && ((compass_diff > event.acceleration.x) || (compass_diff > event.acceleration.y) || (compass_diff > event.acceleration.x * -1) || (compass_diff > event.acceleration.y * -1))) {
    FR_speed = hover_speed + 10;
    BL_speed = hover_speed + 10;
  }
  else {
    FL_speed = FL_speed;
    FR_speed = FR_speed;
    BL_speed = BL_speed;
    BR_speed = BR_speed;
  }

  //Use GPS and barometer to reach cruising altitude before proceeding to destination
  bmp.getEvent(&event);
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

  /*if(home_status = r) {
    
  }*/
  
  if(((bmp.pressureToAltitude(seaLevelPressure,event.pressure))/feetConversion) < cruisingAltitude) {
    ascend();
  } 
 //Once cruising altitude is reached, magnometer will be used to point in the direction of the destination
  else if((((bmp.pressureToAltitude(seaLevelPressure,event.pressure))/feetConversion) > cruisingAltitude)) {
    compass_angle = lat_lon_bearing;
    if(distance_in_ft(currentCoord[0], currentCoord[1], fireCoord[0], fireCoord[1]) > 25) {
      compass_angle;
      forward();
    }
    else if(distance_in_ft(currentCoord[0], currentCoord[1], fireCoord[0], fireCoord[1]) < 25){
      if(((bmp.pressureToAltitude(seaLevelPressure,event.pressure))/feetConversion) > recon_alt) {
      descend();
      }
      else if((((bmp.pressureToAltitude(seaLevelPressure,event.pressure))/feetConversion) < recon_alt + recon_alt_buffer) && (((bmp.pressureToAltitude(seaLevelPressure,event.pressure))/feetConversion) > recon_alt - recon_alt_buffer)) {
      flight_square();    
      }
  }
  }

  //Avoid obstacles detected by ultrasonic sensors
  //front ultrasonic sensor
  long durationF, distanceF;
  digitalWrite(trigPinF, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinF, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinF, LOW);
  durationF = pulseIn(echoPinF, HIGH);
  distanceF = (durationF/2) / 29.1;

  //back ultrasonic sensor
  long durationB, distanceB;
  digitalWrite(trigPinB, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinB, LOW);
  durationB = pulseIn(echoPinB, HIGH);
  distanceB = (durationB/2) / 29.1;

  //left ultrasonic sensor
  long durationL, distanceL;
  digitalWrite(trigPinL, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinL, LOW);
  durationL = pulseIn(echoPinL, HIGH);
  distanceL = (durationL/2) / 29.1;
  
  //right ultrasonic sensor
  long durationR, distanceR;
  digitalWrite(trigPinR, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH);
  distanceR = (durationR/2) / 29.1;
  
 if((distanceF < obj_dist) && (distanceF < distanceB) && (distanceF < distanceL) && (distanceF < distanceR)) {
  reverse();
 }
 else if((distanceB < obj_dist) && (distanceB < distanceF) && (distanceB < distanceL) && (distanceB < distanceR)) {
  forward();
 }
 else if((distanceL < obj_dist) && (distanceL < distanceB) && (distanceL < distanceF) && (distanceL < distanceR)) {
  right();
 }
 else if((distanceR < obj_dist) && (distanceR < distanceB) && (distanceR < distanceL) && (distanceR < distanceF)) {
  left();
 }
 //Inferno will then transmit sensor data and video feed to firefighter's mobile device

}
