#include <max6675.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "wire.h"

int buffersize=2000;
int acel_deadzone = 8;
int giro_deadzone = 1;

MPU6050 mpu;
Max6675 ts(4, 5, 6);

int16_t ax, ay, az, gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

int main_state = 0;
int POWER = A0;
int MOSFET1 = A1;
int MOSFET2 = A2;
int count = 0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
#define LED_BUILTIN 8
#define LED_CHECK_POWER 9
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];  
float pre_ypr[3];
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 
void setup()
{
  
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Yaw,Roll,Pitch,Cur,Vol,Temp");
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  ts.setOffset(0);
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // LED 핀 설정 
  pinMode(LED_CHECK_POWER, OUTPUT);

  //mpu.setIntFIFOBufferOverflowEnabled(0);
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
  }
  
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);

  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
   Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
  pinMode(LED_PIN, OUTPUT);


  // set offset for temperature measurement.
  // 1 stannds for 0.25 Celsius
  digitalWrite(MOSFET1,1);
  delay(500);
  digitalWrite(MOSFET2,1);
//  Serial.print("Cur ");
//  Serial.print("Vol ");
//  Serial.println("Cel");
}
 
void loop()
{
    
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize){}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
    
  fifoCount = mpu.getFIFOCount();
 
  switch(main_state){
    case 0:
      get_pos();// yaw roll pitch 구하여 출력 시키는 함수
      print_val(); // cur, vol, temp 받아서 출력 시키는 함수. 
      digitalWrite(LED_CHECK_POWER, HIGH);
      if(ts.getCelsius() > 90.0){
        digitalWrite(MOSFET1,0);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(MOSFET2,0); // 온도가 90도 이상이 되면 병렬 >> 직렬 전환. 
        digitalWrite(LED_BUILTIN, LOW); 
       
        pre_ypr[0] = ypr[0] * 180/M_PI;
        pre_ypr[1] = ypr[1] * 180/M_PI;
        pre_ypr[2] = ypr[2] * 180/M_PI; 
        
        main_state++;
        break;
      }
      else
        break;

    case 1:
      get_pos();
      print_val();
      if(abs((ypr[0] * 180/M_PI) - pre_ypr[0]) > 10 || abs((ypr[1] * 180/M_PI) - pre_ypr[1]) > 10 || abs((ypr[2] * 180/M_PI) - pre_ypr[2]) > 10){
        count = 0;
      }
      else{
        count++;
      }

      if(count == 20 ){
        main_state++;  
        break;
      }
      if(ts.getCelsius() > 90.0){
        digitalWrite(POWER,1);
        digitalWrite(LED_CHECK_POWER, LOW);
      }
      else if(ts.getCelsius()< 90.0){
        digitalWrite(POWER,0);
        digitalWrite(LED_CHECK_POWER, HIGH);
      }

      pre_ypr[0] = ypr[0] * 180/M_PI;
      pre_ypr[1] = ypr[1] * 180/M_PI;
      pre_ypr[2] = ypr[2] * 180/M_PI;
      break;

    case 2:
      digitalWrite(POWER,1);
      digitalWrite(LED_CHECK_POWER, LOW);
      Serial.println("Turn off!");
      get_pos();
      print_val();
      break;
  }
  delay(500);
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void get_pos(){
  if (mpuIntStatus & 0x02) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
      
      mpu.getFIFOBytes(fifoBuffer, packetSize);
        
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
       // *============* yaw, roll, pitch 출력부 *============*//
        Serial.println("yaw      pitch     roll    cur    vol    temp ");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("   ");
        Serial.print(',');
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("   ");
        Serial.print(',');
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print("   ");
        Serial.print(',');
       #endif

      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
}

void print_val(){
  int val0 = analogRead(POWER);
  int val1 = analogRead(MOSFET1);
  int val2 = analogRead(MOSFET2);
  int current = analogRead(A3);
  int voltage = analogRead(A4);
  //*============* cur, vol, temp 출력부 *============*//
  Serial.print(current);
  Serial.print("   ");
  Serial.print(',');
  Serial.print(voltage);
  Serial.print("   ");
  Serial.print(',');
  Serial.println(ts.getCelsius(), 2);
}
void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}
