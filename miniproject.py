##DONE IN ARDUINO C/C++

## transmitter code

#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

Servo sg90;
int servo_pin = 18;

MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;

bool motionDetected = false;
unsigned long lastMotionTime = 0;
const int motionTimeout = 3000;  // 3 sec of no motion before lifting
const int motionThreshold = 12000;  // Ignore small vibrations
const int minChangeThreshold = 3000;  // Only detect large changes

// Store previous values for comparison
int16_t prevAx = 0, prevAy = 0, prevAz = 0;

void setup() {
    Serial.begin(115200);
    sg90.attach(servo_pin, 500, 2400);
    Wire.begin(21, 22);
    
    Serial.println("Initializing MPU6050...");
    sensor.initialize();
    
    if (sensor.testConnection()) {
        Serial.println("MPU6050 Connected!");
    } else {
        Serial.println("MPU6050 Connection Failed! Check wiring.");
    }

    sg90.write(0);
    Serial.println("System Ready");
}

void loop() {
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate absolute differences
    int diffAx = abs(ax - prevAx);
    int diffAy = abs(ay - prevAy);
    int diffAz = abs(az - prevAz);

    // Update previous values
    prevAx = ax;
    prevAy = ay;
    prevAz = az;

    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" | AY: "); Serial.print(ay);
    Serial.print(" | AZ: "); Serial.println(az);
    
    Serial.print("ΔAX: "); Serial.print(diffAx);
    Serial.print(" | ΔAY: "); Serial.print(diffAy);
    Serial.print(" | ΔAZ: "); Serial.println(diffAz);

    // Detect significant motion only
    bool newMotionState = (diffAx > minChangeThreshold || diffAy > minChangeThreshold || diffAz > minChangeThreshold);

    if (newMotionState) {
        lastMotionTime = millis();
        if (!motionDetected) {
            Serial.println("⚡ Large Motion Detected! Keeping iron down.");
            sg90.write(0);  // Iron in normal position
            motionDetected = true;
        }
    } 
    else if (millis() - lastMotionTime > motionTimeout) {
        if (motionDetected) {
            Serial.println("🚨 No large motion for 3 sec! Lifting iron.");
            sg90.write(90);  // Lift iron up
            motionDetected = false;
        }
    }

    delay(200);
}

## reciever code
#include <esp_now.h>
#include <WiFi.h>


int count = 0;
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.println();


  if(myData.b == 1){
    digitalWrite(2,HIGH);
    count =0;
    
  }

  if(myData.b == 0){
    digitalWrite(2,LOW);
    count =0;
    
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(2,OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

  count++;
  delay(1000);


  if(count >= 10){

    digitalWrite(2,LOW);
    count = 0;

  }

}
