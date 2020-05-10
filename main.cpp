// Editor PRAGYA VARSHNEY
#include <painlessMesh.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <LoRa.h>
#include <edushield.h>



// some gpio pin that is connected to an LED...
// on my rig, this is 5, change to the right number of your LED.
//#define   LED             25      // GPIO number of connected LED, ON ESP-12 IS GPIO2

#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for

#define   MESH_SSID       "MyESPG4"
#define   MESH_PASSWORD   "testpassword"
#define   MESH_PORT       5555
Adafruit_BME680 bme; // I2C
 #define PIR 13   // PIR sensor 2
 #define RELAY1 27
 #define RELAY2 14
#define LED 25
int counter = 0;
int count1=0;
 float temp;
float temp1;
float Avg=20.0;
float Sum=0;
int node;
int node1;
#define Setpoint  19
#define range  2
/////////////////// Timer stuff
volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

// Prototypes
void sendMessage(); 
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

void setup() {
  Serial.begin(115200);
  //lora start
  SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_CS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //lora end
   // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  //bme.setHumidityOversampling(BME680_OS_2X);
  
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  //pinMode(LED, OUTPUT);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  //mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION | COMMUNICATION);  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION);  // set before init() so that you can see startup messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();

  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
      // If on, switch off, else switch on
      if (onFlag)
        onFlag = false;
      else
        onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);

      if (blinkNoNodes.isLastIteration()) {
        // Finished blinking. Reset task for next run 
        // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
        
        // Calculate delay based on current mesh time and BLINK_PERIOD
        // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD - 
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
      }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();

  randomSeed(analogRead(A0));


  pinMode(LED,OUTPUT); // Sets the LED as an output
  pinMode(RELAY1,OUTPUT); // Sets RELAY as an output
  pinMode(RELAY2,OUTPUT); // Sets RELAY as an output
  
  // Add your timer interrupt here
 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 60000000, true);// 60 sec interrupt
  timerAlarmEnable(timer);
}


void loop() {
 // int minat = 0;
  //int maxat = 0;
  userScheduler.execute(); // it will run mesh scheduler as well
  mesh.update();
  digitalWrite(LED, !onFlag);
  unsigned long endTime = bme.beginReading();
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());
  Serial.println("\n");
   node = mesh.getNodeList().size();
   node1= nodes.size();
  Serial.printf("Node:%d \n",node);
  Serial.printf("Node:%d \n",node1);
  
  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

   

   if (interruptCounter == 1) {

 
    totalInterruptCounter++;
 
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
    Serial.print("An interrupt as occurred. Time: ");
    Serial.print(millis());
    Serial.print("\n");
    if (node==2){
    Sum= temp+temp1+(bme.temperature);
    Serial.printf("Sum temp= %f ",Sum);
   
    Avg= Sum/(3);
    Serial.printf("Avg temp= %f ",Avg);//Average temperature
    }
    //lora 
     Serial.print("Sending packet: ");
     Serial.println(count1);

  // send packet
  LoRa.beginPacket();
  LoRa.print(Avg);
   LoRa.print("//");
  LoRa.print(count1);
  LoRa.endPacket();

  count1++;

    //lora

    //output
   bool motion = digitalRead(PIR); // Read the PIR
    if (motion) {
        if (Avg>=(Setpoint+range))  
           {
            digitalWrite(RELAY1,HIGH);
            digitalWrite(LED,HIGH);
           Serial.println("AC on\n");
           } 
        else {
           digitalWrite(RELAY1,LOW);
           digitalWrite(LED,LOW);
           Serial.println("AC off\n");
             }       
        if (Avg<=(Setpoint-range))
           {
             digitalWrite(RELAY2,HIGH);
              Serial.println("Heater on\n");

            }
        else {
           digitalWrite(RELAY2,LOW);
           Serial.println("Heater off\n");
            } 
         }  

     else{
       digitalWrite(RELAY2,LOW);
       Serial.println("Heater off\n");
       digitalWrite(RELAY1,LOW);
       Serial.println("AC off\n");
       digitalWrite(LED,LOW);

         }
      counter++;
    portENTER_CRITICAL(&timerMux);
    interruptCounter = 0;
    portEXIT_CRITICAL(&timerMux);


 } 

  
}

void sendMessage() {
 // String msg = String(Avg);
  String msg = "Hello from node bme ";
  msg += mesh.getNodeId();
  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
  msg += " Temprature:" + String(Avg) + "*C";
  //msg += " Humidity:" + String(bme.humidity) + "%";*/
  mesh.sendBroadcast(msg);

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  Serial.printf("Sending message: %s\n", msg.c_str());
  
  taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
}


void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  std::string value = msg.c_str();
  uint32_t id1 = from;
  
  Serial.println(id1);
  if (id1== 2754425633){
   temp=atof(value.c_str());
    
   Serial.printf("temprature1= %f ",temp);
   
   
   }
  else if (id1== 2754425765){
    temp1=atof(value.c_str());
  
   Serial.printf("temprature2= %f ",temp1);
   }
   

   
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}