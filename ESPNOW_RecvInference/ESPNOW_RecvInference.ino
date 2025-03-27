#include <esp_now.h>
#include <WiFi.h>
#define FORWARD 17
#define BACKWARD 19
#define IDLE 21
#define LEFT 22
#define RIGHT 23

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool forward;
  bool backward;
  bool left;
  bool idle;
  bool right;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  if(myData.forward){
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACKWARD, LOW);
    digitalWrite(IDLE, LOW);
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, LOW);
  }
  else if(myData.backward){
    digitalWrite(BACKWARD, HIGH);
    digitalWrite(FORWARD, LOW);
    digitalWrite(IDLE, LOW);
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, LOW);
  }
  else if(myData.idle){
    digitalWrite(IDLE, HIGH);
    digitalWrite(BACKWARD, LOW);
    digitalWrite(FORWARD, LOW);
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, LOW);
  }
  else if(myData.left){
    digitalWrite(LEFT, HIGH);
    digitalWrite(BACKWARD, LOW);
    digitalWrite(IDLE, LOW);
    digitalWrite(FORWARD, LOW);
    digitalWrite(RIGHT, LOW);
  }
  else if(myData.right){
    digitalWrite(RIGHT, HIGH);
    digitalWrite(BACKWARD, LOW);
    digitalWrite(IDLE, LOW);
    digitalWrite(LEFT, LOW);
    digitalWrite(FORWARD, LOW);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  pinMode(IDLE, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    digitalWrite(RIGHT, LOW);
    digitalWrite(BACKWARD, LOW);
    digitalWrite(IDLE, LOW);
    digitalWrite(LEFT, LOW);
    digitalWrite(FORWARD, LOW);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}