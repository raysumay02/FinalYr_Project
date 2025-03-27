#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Gesture_Control_inferencing.h>
#include <esp_now.h>
#include <WiFi.h>

#define FREQUENCY_HZ        60
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x94, 0xE6, 0x86, 0x3B, 0x79, 0xA8};

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

esp_now_peer_info_t peerInfo;

// objeto da classe Adafruit_MPU6050
Adafruit_MPU6050 mpu;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

static unsigned long last_interval_ms = 0;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);

  Serial.print("Features: ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.print("Label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

}

void loop() {
  sensors_event_t a, g, temp;

  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

    mpu.getEvent(&a, &g, &temp);

    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;

    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      Serial.println("Running the inference...");
      signal_t signal;
      ei_impulse_result_t result;
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }

      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

      if (res != 0) return;

      ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms.)",
                result.timing.dsp, result.timing.classification);
      ei_printf(": \n");

      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        if (result.classification[ix].value > 0.75){
          if(result.classification[ix].label == "Forward")
          {
            myData.forward = true;
            myData.backward = false;
            myData.idle = false;
            myData.left = false;
            myData.right = false;
          }
          else if(result.classification[ix].label == "Backward"){
            myData.backward = true;
            myData.forward = false;
            myData.idle = false;
            myData.left = false;
            myData.right = false;
          }
          else if(result.classification[ix].label == "Idle"){
            myData.idle = true;
            myData.backward = false;
            myData.forward = false;
            myData.left = false;
            myData.right = false;
          }
          else if(result.classification[ix].label == "Left"){
            myData.left = true;
            myData.backward = false;
            myData.forward = false;
            myData.idle = false;
            myData.right = false;
          }
          else if(result.classification[ix].label == "Right"){
            myData.right = true;
            myData.backward = false;
            myData.forward = false;
            myData.idle = false;
            myData.left = false;
            
          }
          else{
            myData.forward = false;
            myData.backward = false;
            myData.idle = false;
            myData.left = false;
            myData.right = false;
          }
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

            if (result == ESP_OK) {
              Serial.println("Sent with success");
          }
            else {
              Serial.println("Error sending the data");
          }
        }
      }
      feature_ix = 0;
    }
  }
  // Send message via ESP-NOW
  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}




