#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <Preferences.h>
#include "secrets.h"

/************ Inspection API ************/
const char* apiUrl="https://opufcp3ut6.execute-api.ap-south-1.amazonaws.com/inspect";
const char* bucket="esp32-solarimg";
const char* imgKey="current-panel.jpg";

/************ Receiver ESP32-CAM MAC ************/
uint8_t camMAC[] = {0x84,0x1F,0xE8,0x68,0x8F,0xA4};

/************ Relay ESP32 MAC ************/
uint8_t relayMAC[] = {0x88,0x57,0x21,0x78,0xE7,0x80};

/************ MQTT ************/
const char* DEVICE_ID = "ESP32-Solar-Controller";
const char* CONTROL_TOPIC = "solar/panel/control";
const char* STATUS_TOPIC = "solar/panel/status";
const char* MOTOR_TOPIC = "solar/panel/motor";
const char* INSPECTION_TOPIC = "solar/panel/inspection";
const char* ALERT_TOPIC = "solar/panel/alert";

/************ MOTOR PINS *************/
#define STEP_PIN 5
#define DIR_PIN  6

/************ MOTOR SPEED SETTINGS ************/
int startDelay = 2500;   // gentle start
int runDelay   = 300;    // fast speed
int accelStep  = 30;     // acceleration increment

int slowStartDelay = 3500;   // slower gentle start
int slowRunDelay   = 800;    
int slowAccelStep  = 20;     

/************ MOTOR DISTANCE SETTINGS ************/
#define STEPS_PER_CM 400  

/************ MOTOR STEP COUNTER ************/
volatile long stepCounter = 0;

/************ Objects ************/
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences prefs;

/************ State ************/
bool systemState = false;
bool waitingForUpload = false;
unsigned long captureStartTime = 0;
unsigned long lastTrigger = 0;
unsigned long lastRelayHello = 0;
const unsigned long triggerInterval = 15000;
const unsigned long uploadTimeout = 50000;

/**************** AWS IoT LOGGING FUNCTIONS ****************/
void publishMotorStatus(const char* action, int distance, const char* direction, const char* speed = "normal") {
  if(!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["device"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["action"] = action;
  doc["distance_cm"] = distance;
  doc["direction"] = direction;
  doc["speed"] = speed;
  doc["step_count"] = stepCounter;
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(MOTOR_TOPIC, buffer);
}

void publishInspectionResult(JsonObject& result, int attempt = 0) {
  if(!mqttClient.connected()) return;
  StaticJsonDocument<512> doc;
  doc["device"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["object_above"] = result["object_above"] | false;
  doc["object_type"] = result["object_type"] | "none";
  doc["severity"] = result["severity"] | "unknown";
  doc["action"] = result["action"] | "none";
  doc["confidence"] = result["confidence"] | 0;
  if(attempt > 0) doc["clearance_attempt"] = attempt;
  char buffer[512];
  serializeJson(doc, buffer);
  mqttClient.publish(INSPECTION_TOPIC, buffer);
}

void publishAlert(const char* alertType, const char* message, const char* severity = "warning") {
  if(!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["device"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["alert_type"] = alertType;
  doc["message"] = message;
  doc["severity"] = severity;
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(ALERT_TOPIC, buffer);
  mqttClient.loop();
}

/**************** MOTOR FUNCTIONS ****************/
void stepMotor(int delayUs) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayUs);
  stepCounter++;

  // Send step update to relay ESP32
  char stepMsg[32];
  snprintf(stepMsg, sizeof(stepMsg), "STEP:%ld", stepCounter);
  esp_now_send(relayMAC, (uint8_t*)stepMsg, strlen(stepMsg));
}

long calculateAccelSteps() {
  long count = 0;
  for (int d = startDelay; d > runDelay; d -= accelStep) count++;
  return count;
}

void moveExactCM(int distanceCM) {
  stepCounter = 0;
  long totalSteps = distanceCM * STEPS_PER_CM;
  long accelSteps = calculateAccelSteps();
  long cruiseSteps = totalSteps - (2 * accelSteps);
  if (cruiseSteps < 0) cruiseSteps = 0;

  for (int d = startDelay; d > runDelay; d -= accelStep) stepMotor(d);
  for (long i = 0; i < cruiseSteps; i++) stepMotor(runDelay);
  for (int d = runDelay; d < startDelay; d += accelStep) stepMotor(d);
}

void moveExactCMSlow(int distanceCM) {
  stepCounter = 0;
  long totalSteps = distanceCM * STEPS_PER_CM;
  long accelSteps = 0;
  for (int d = slowStartDelay; d > slowRunDelay; d -= slowAccelStep) accelSteps++;
  long cruiseSteps = totalSteps - (2 * accelSteps);
  if(cruiseSteps < 0) cruiseSteps = 0;

  for(int d = slowStartDelay; d > slowRunDelay; d -= slowAccelStep) stepMotor(d);
  for(long i=0;i<cruiseSteps;i++) stepMotor(slowRunDelay);
  for(int d=slowRunDelay; d<slowStartDelay; d+=slowAccelStep) stepMotor(d);
}

/**************** CAPTURE FUNCTION ****************/
void performCapture() {
  Serial.println("🔧 MOTOR: Moving forward 15 CM");
  publishMotorStatus("moving",15,"forward");
  digitalWrite(DIR_PIN,HIGH);
  moveExactCM(15);
  Serial.println("✅ MOTOR: Reached capture position");
  publishMotorStatus("reached",15,"forward");
  delay(500);

  esp_now_send(camMAC,(uint8_t*)"CAPTURE",8);
  Serial.println("📤 Sent → CAPTURE");

  StaticJsonDocument<128> captureDoc;
  captureDoc["device"] = DEVICE_ID;
  captureDoc["action"] = "capture_command_sent";
  captureDoc["timestamp"] = millis();
  char capBuf[128];
  serializeJson(captureDoc, capBuf);
  mqttClient.publish(STATUS_TOPIC, capBuf);
  delay(1000);

  Serial.println("🔧 MOTOR: Moving back 15 CM");
  publishMotorStatus("moving",15,"backward");
  digitalWrite(DIR_PIN,LOW);
  moveExactCM(15);
  Serial.println("✅ MOTOR: Returned to initial position");
  publishMotorStatus("reached",15,"backward");

  waitingForUpload = true;
  captureStartTime = millis();
}

/**************** OBJECT ABOVE HANDLER ****************/
void handleObjectAbove() {
  Serial.println("\n🚨 OBJECT ABOVE DETECTED - Starting clearance sequence");
  prefs.begin("sender", true);
  int attempt = prefs.getInt("objectAttempt", 0);
  prefs.end();

  attempt++;
  Serial.printf("\n═══ OBJECT CLEARANCE - Attempt %d/3 ═══\n", attempt);
  char attemptMsg[128];
  snprintf(attemptMsg,sizeof(attemptMsg),"Starting object clearance attempt %d of 3",attempt);
  publishAlert("object_clearance",attemptMsg,"warning");

  Serial.println("🔧 MOTOR: Moving forward 30 CM");
  publishMotorStatus("clearance_forward",30,"forward","normal");
  digitalWrite(DIR_PIN,HIGH);
  moveExactCM(30);
  Serial.println("✅ MOTOR: Reached 30 CM");
  publishMotorStatus("clearance_reached",30,"forward","normal");
  delay(500);

  Serial.println("🔧 MOTOR: Moving back 30 CM");
  publishMotorStatus("clearance_backward",30,"backward","normal");
  digitalWrite(DIR_PIN,LOW);
  moveExactCM(30);
  Serial.println("✅ MOTOR: Returned to start");
  publishMotorStatus("clearance_complete",30,"backward","normal");
  delay(1000);

  performCapture();

  prefs.begin("sender", false);
  if(attempt>=3){
    prefs.putBool("objectFailed",true);
    prefs.putInt("objectAttempt",0);
  } else {
    prefs.putInt("objectAttempt",attempt);
  }
  prefs.putBool("callAPI",true);
  prefs.end();

  Serial.printf("⏳ Waiting for upload to complete (Attempt %d/3)...\n",attempt);
}

/**************** DIRTY REGION HANDLER ****************/
void handleDirtyRegion() {
  Serial.println("\n🧼 DIRTY REGION DETECTED - Starting cleaning sequence");

  prefs.begin("sender", true);
  int dirtyAttempt = prefs.getInt("dirtyAttempt",0);
  prefs.end();

  dirtyAttempt++;
  Serial.printf("\n═══ DIRTY REGION CLEANING - Attempt %d/3 ═══\n", dirtyAttempt);

  char dirtyMsg[128];
  snprintf(dirtyMsg,sizeof(dirtyMsg),"Starting dirty region cleaning attempt %d of 3",dirtyAttempt);
  publishAlert("dirty_region",dirtyMsg,"warning");

  Serial.println("🧹 MOTOR: Moving forward 30 CM");
  publishMotorStatus("dirty_forward",30,"forward","normal");
  digitalWrite(DIR_PIN,HIGH);
  moveExactCM(30);
  Serial.println("✅ MOTOR: Reached 30 CM");
  publishMotorStatus("dirty_reached",30,"forward","normal");
  delay(500);

  Serial.println("🧹 MOTOR: Moving back 30 CM");
  publishMotorStatus("dirty_backward",30,"backward","normal");
  digitalWrite(DIR_PIN,LOW);
  moveExactCM(30);
  Serial.println("✅ MOTOR: Returned to start");
  publishMotorStatus("dirty_complete",30,"backward","normal");
  delay(1000);

  performCapture();

  prefs.begin("sender", false);
  if(dirtyAttempt>=3){
    prefs.putBool("dirtyFailed",true);
    prefs.putInt("dirtyAttempt",0);
  } else {
    prefs.putInt("dirtyAttempt",dirtyAttempt);
  }
  prefs.putBool("callAPI",true);
  prefs.end();

  Serial.printf("⏳ Waiting for upload to complete (Attempt %d/3)...\n",dirtyAttempt);
}

/**************** CALL INSPECTION API ****************/
void callInspectionAPI() {
  if(WiFi.status()!=WL_CONNECTED) return;

  HTTPClient http;
  http.begin(apiUrl);
  http.addHeader("Content-Type","application/json");
  http.setTimeout(30000);

  StaticJsonDocument<200> doc;
  doc["bucket"] = bucket;
  doc["key"] = imgKey;
  String body;
  serializeJson(doc,body);

  Serial.println("🤖 Calling Inspection API...");
  unsigned long start = millis();
  int code = http.POST(body);
  Serial.printf("🌐 Status: %d (%.1fs)\n", code,(millis()-start)/1000.0);

  if(code==200){
    String response = http.getString();
    Serial.println(response);

    // Send API response to relay ESP32
    esp_now_send(relayMAC, (uint8_t*)response.c_str(), response.length());
    Serial.println("📤 Sent API response to Relay ESP32");

    DynamicJsonDocument result(1024);
    if(!deserializeJson(result,response) && result.containsKey("result")){
      JsonObject res = result["result"];
      bool objectAbove = res["object_above"] | false;
      bool cleaningRequired = res["cleaning_required"] | false;

      Serial.println("\n========== INSPECTION RESULTS ==========");
      Serial.printf("Object Above: %s\n", objectAbove?"YES ⚠️":"NO ✓");
      Serial.printf("Dirty Cleaning: %s\n", cleaningRequired?"YES ⚠️":"NO ✓");
      Serial.println("========================================");

      prefs.begin("sender",true);
      bool objectFailed = prefs.getBool("objectFailed",false);
      bool dirtyFailed = prefs.getBool("dirtyFailed",false);
      int currentObjectAttempt = prefs.getInt("objectAttempt",0);
      int currentDirtyAttempt = prefs.getInt("dirtyAttempt",0);
      prefs.end();

      publishInspectionResult(res,currentObjectAttempt);

      // HANDLE OBJECT ABOVE
      if(objectAbove && !objectFailed){
        Serial.printf("\n🔄 Object still detected - Will retry (Current: %d/3)\n",currentObjectAttempt);
        handleObjectAbove();
        return;
      }

      // OBJECT CLEARED
      if(!objectAbove && currentObjectAttempt>0){
        char msg[128];
        snprintf(msg,sizeof(msg),"Object cleared after %d attempt(s)",currentObjectAttempt);
        publishAlert("object_cleared",msg,"info");
        Serial.println("\n✅ OBJECT CLEARED SUCCESSFULLY!");
        prefs.begin("sender",false);
        prefs.putInt("objectAttempt",0);
        prefs.end();
      }

      // HANDLE DIRTY REGION
      if(cleaningRequired && !dirtyFailed){
        Serial.printf("\n🔄 Dirty region detected - Will retry (Current: %d/3)\n",currentDirtyAttempt);
        handleDirtyRegion();
        return;
      }

      // DIRTY CLEARED
      if(!cleaningRequired && currentDirtyAttempt>0){
        char msg[128];
        snprintf(msg,sizeof(msg),"Dirty region cleared after %d attempt(s)",currentDirtyAttempt);
        publishAlert("dirty_region_cleared",msg,"info");
        Serial.println("\n✅ DIRTY REGION CLEARED SUCCESSFULLY!");
        prefs.begin("sender",false);
        prefs.putInt("dirtyAttempt",0);
        prefs.end();
      }

      // WAIT BEFORE NEXT CAPTURE
      Serial.println("\n⏳ Waiting 10 seconds before next capture...");
      delay(10000);
      Serial.println("✅ Ready for next capture\n");
    }
  } else {
    Serial.printf("❌ API Error: %d\n", code);
    String errorResponse = http.getString();
    Serial.println(errorResponse);
    publishAlert("api_error",errorResponse.c_str(),"error");
  }

  http.end();
}

/**************** MQTT & ESP-NOW CALLBACKS ****************/
void mqttCallback(char* topic, byte* payload, unsigned int length){
  StaticJsonDocument<128> doc;
  if(deserializeJson(doc,payload,length)) return;

  bool newState = systemState;
  if(doc.containsKey("state")){
    const char* s = doc["state"];
    if(strcasecmp(s,"on")==0) newState=true;
    else if(strcasecmp(s,"off")==0) newState=false;
  }
  if(doc.containsKey("command")){
    int cmd = doc["command"];
    if(cmd==1) newState=true;
    else if(cmd==0) newState=false;
  }

  if(newState!=systemState){
    systemState=newState;
    prefs.begin("system",false);
    prefs.putBool("on",systemState);
    prefs.end();
    Serial.println(systemState?"✅ SYSTEM ON":"⏸️ SYSTEM OFF");

    StaticJsonDocument<64> s;
    s["state"] = systemState?"on":"off";
    char buf[64];
    serializeJson(s,buf);
    mqttClient.publish(STATUS_TOPIC,buf);
  }
}

void onRecv(const uint8_t *mac, const uint8_t *data, int len){
  String msg="";
  for(int i=0;i<len;i++) msg+=char(data[i]);
  Serial.println("📥 Camera → " + msg);

  if(msg=="UPLOAD_SUCCESS"){
    if(systemState){
      Serial.println("🟢 Upload Success - Restarting for API");
      prefs.begin("sender",false);
      prefs.putBool("callAPI",true);
      prefs.end();
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("💤 Upload Success - But system is OFF");
      waitingForUpload=false;
    }
  } else if(msg=="UPLOAD_FAILED"){
    Serial.println("🔴 Upload Failed");
    publishAlert("upload_failed","Image upload to S3 failed","error");
    waitingForUpload=false;
  } else if(msg=="CAPTURE_OK"){
    Serial.println("📸 Capture acknowledged");
  }
}

/**************** SEND CAPTURE ****************/
void sendCapture(){
  Serial.println("\n📤 Starting capture sequence...");
  performCapture();
}

/**************** SETUP ****************/
void setup(){
  Serial.begin(115200);
  delay(1000);

  pinMode(STEP_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  Serial.println("🔧 Motor pins initialized");

  prefs.begin("system",true);
  systemState = prefs.getBool("on",false);
  prefs.end();

  prefs.begin("sender",true);
  bool shouldCallAPI = prefs.getBool("callAPI",false);
  prefs.end();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  // Initialize ESP-NOW before WiFi connection (for sending status)
  if(esp_now_init()==ESP_OK){
    esp_now_register_recv_cb(onRecv);
    
    // Add camera peer
    esp_now_peer_info_t camPeer={};
    memcpy(camPeer.peer_addr,camMAC,6);
    esp_now_add_peer(&camPeer);
    
    // Add relay peer
    esp_now_peer_info_t relayPeer={};
    memcpy(relayPeer.peer_addr,relayMAC,6);
    esp_now_add_peer(&relayPeer);
    
    Serial.println("📡 ESP-NOW initialized (Camera + Relay)");
  }

  // Send STARTING_WIFI to relay ESP32 before WiFi connection
  esp_now_send(relayMAC, (uint8_t*)"STARTING_WIFI", 13);
  Serial.println("📤 Sent STARTING_WIFI to Relay ESP32");
  delay(100);

  Serial.print("📡 WiFi");
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while(WiFi.status()!=WL_CONNECTED){ Serial.print("."); delay(300);}
  Serial.println(" ✔");
  
  // Send WiFi success message to relay ESP32
  esp_now_send(relayMAC, (uint8_t*)"WIFI_CONNECTED", 15);
  Serial.println("📤 Sent WIFI_CONNECTED to Relay ESP32");
  delay(100);

  wifiClient.setCACert(AWS_ROOT_CA);
  wifiClient.setCertificate(DEVICE_CERT);
  wifiClient.setPrivateKey(DEVICE_PRIVATE_KEY);
  mqttClient.setServer(AWS_IOT_ENDPOINT,AWS_IOT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.print("☁️ AWS");
  while(!mqttClient.connected()){
    if(mqttClient.connect(DEVICE_ID)){
      Serial.println(" ✔");
      mqttClient.subscribe(CONTROL_TOPIC);
    } else { Serial.print("."); delay(1000);}
  }

  if(shouldCallAPI){
    prefs.begin("sender",false);
    prefs.putBool("callAPI",false);
    prefs.end();

    if(systemState){
      Serial.println("🔄 API CALL PENDING - Executing...");
      callInspectionAPI();
    } else {
      Serial.println("💤 API skipped - System is OFF");
    }
  }

  Serial.print("\n══════ READY (");
  Serial.print(systemState?"ON":"OFF");
  Serial.println(") ══════\n");
}
   
/**************** Loop ****************/
void loop(){
  // Keep AWS connected (priority)
  if(!mqttClient.connected()){
    if(mqttClient.connect(DEVICE_ID)){
      mqttClient.subscribe(CONTROL_TOPIC);
    }
  }
  mqttClient.loop();

  // Send HELLO to relay ESP32 every 1 second
  if(millis() - lastRelayHello >= 1000){
    lastRelayHello = millis();
    esp_now_send(relayMAC, (uint8_t*)"HELLO", 6);
    Serial.println("📤 Sent HELLO to Relay ESP32");
  }

  // Only run if ON
  if(systemState){
    if(!waitingForUpload && millis() - lastTrigger >= triggerInterval){
      lastTrigger = millis();
      sendCapture();
    }
    if(waitingForUpload && millis() - captureStartTime > uploadTimeout){
      Serial.println("⏱️ Timeout");
      publishAlert("upload_timeout", "Image upload timed out", "warning");
      waitingForUpload = false;
      lastTrigger = millis();
    }
  }
  delay(50);
}