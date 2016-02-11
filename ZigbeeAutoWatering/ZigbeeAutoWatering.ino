#include <Timer.h>
#include <XBee.h>
#include <Printers.h>

#include <AutoWatering.h>
#include <DHT.h>

#define MaxTemprature 40  //The Maximum Value of the Temperature
#define Sensor 1
#define Carbon 0

boolean isDebugEnabled=true;
boolean isDebugXbeeEnabled=false;

boolean doUpdate = false;
boolean doPumpUpdate = false;
boolean doGetReads = true;
boolean doCheckEnv = false;
boolean existReads = false;
boolean doToggleOn = false;

Timer t;
int trafficCop =0;  //tracks which station has the right of way (is on)

long stationTime[] = {0,0,0,0,0,0,0,0,0};
int8_t stationTimer[] = {0,0,0,0,0,0,0,0,0}; 
int queue[]={0,0,0,0,0,0,0,0,0};  // off: 0, queued: 1, running: 2

boolean isConfigPump = false; // if true, relay8 is used as the pump/master valve switch.  Can be toggled by device tye v2.7 and later

AutoWatering flower;

int temperature;
int moisture_dat;
int humidity;

// create the XBee object
XBeeWithCallbacks xbee;

#define PACKED_STRUCT struct __attribute__((__packed__))

typedef PACKED_STRUCT cluster_cmd_req_header_t {
  uint8_t   frame_control;
  uint8_t   sequence_number;
  uint8_t   command;
} cluster_cmd_req_header_t;

void zbReceive(ZBRxResponse& rx, uintptr_t) {
  // Create a reply packet containing the same data
  // This directly reuses the rx data array, which is ok since the tx
  // packet is sent before any new response is received
  ZBTxRequest tx;
  tx.setAddress64(rx.getRemoteAddress64());
  tx.setAddress16(rx.getRemoteAddress16());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());

  // Send the reply, but do not wait for the tx status reply. If an
  // error occurs, the global onTxStatusResponse handler will print an
  // error message, but no message is printed on succes.
  xbee.send(tx);
  if (isDebugXbeeEnabled) Serial.println(F("Sending ZBTxRequest"));
}

void receive16(Rx16Response& rx, uintptr_t) {
  // Create a reply packet containing the same data
  // This directly reuses the rx data array, which is ok since the tx
  // packet is sent before any new response is received
  Tx16Request tx;
  tx.setAddress16(rx.getRemoteAddress16());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());

  // Send the reply, but do not wait for the tx status reply. If an
  // error occurs, the global onTxStatusResponse handler will print an
  // error message, but no message is printed on succes.
  xbee.send(tx);
  if (isDebugXbeeEnabled) Serial.println(F("Sending Tx16Request"));
}

void receive64(Rx64Response& rx, uintptr_t) {
  // Create a reply packet containing the same data
  // This directly reuses the rx data array, which is ok since the tx
  // packet is sent before any new response is received
  Tx64Request tx;
  tx.setAddress64(rx.getRemoteAddress64());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());

  // Send the reply, but do not wait for the tx status reply. If an
  // error occurs, the global onTxStatusResponse handler will print an
  // error message, but no message is printed on succes.
  xbee.send(tx);
  if (isDebugXbeeEnabled)  Serial.println(F("Sending Tx64Request"));
}
void receiveZBExplicitRx(ZBExplicitRxResponse& rx, uintptr_t) {
  if (isDebugXbeeEnabled) printResponseCb(rx, (uintptr_t)(Print*)&Serial);
  
//  uint8_t *payload_rx = rx.getFrameData() + rx.getDataOffset();

  if (isDebugXbeeEnabled) Serial.println(F("Home Automation Profile"));

  if (rx.getClusterId() == 0x0004){
    if (isDebugXbeeEnabled) Serial.println(F("Simple Descriptor Request"));
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = 0x8004; //rx.getClusterId();
      uint16_t profileId = rx.getProfileId();

      // 7e 
      // 00 2b
      // 11                         frame type
      // 05                         frame id
      // XX XX XX XX XX XX XX XX    64bit addr
      // 00 00                      16bit addr
      // 00                         src endpoint
      // 00                         dest endpoint
      // 80 04                      cluster id
      // 00 00                      profileId
      // 00                         broadcastRadius
      // 00                         transmitOptions
      // payload
      // 1c 
      // 00                         Status
      // 16 8f                      Network Address
      // 12                         Length
      // 01                         Endpoint
      // 04 01                      Application profileID
      // 09 00                      Application device ID
      // 00                         Application device version
      // 05                         Input cluster count
      // 00 00                      Input cluster list
      // 06 00
      // 02 04
      // 05 04
      // 04 0b
      // 00                       Output cluster count
      // 9f                       checksum
      
      uint8_t payload[] = {0x1C,0x00,0x16,0x8F,0x12,0x01,0x04,0x01,0x09,0x00,0x00,0x05,0x00,0x00,0x06,0x00,0x02,0x04,0x05,0x04,0x04,0x0B,0x00};
      
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      
      xbee.send(tx);
  }
  if (rx.getClusterId() == 0x0005){
    if (isDebugXbeeEnabled) Serial.println(F("Active Endpoints Request"));
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = 0x8005; //rx.getClusterId();
      uint16_t profileId = rx.getProfileId();

      // 7e 
      // 00 2b
      // 11                         frame type
      // 05                         frame id
      // XX XX XX XX XX XX XX XX    64bit addr
      // 00 00                      16bit addr
      // 00                         src endpoint
      // 00                         dest endpoint
      // 80 05                      cluster id
      // 00 00                      profileId
      // 00                         broadcastRadius
      // 00                         transmitOptions
      // payload
      // 1b 
      // 00                         Status
      // 16 8f                      Network Address
      // 01                         Active Endpoint Count
      // 01                         Active Endpoint List
      // e8                         checksum

      uint8_t payload[] = {0x1B,0x00,0x16,0x8F,0x01,0x01};
      
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      
      xbee.send(tx);
  }
  
  // BasicCluster
  if (rx.getClusterId() == 0x0000){
    if (isDebugXbeeEnabled) Serial.println(F("BasicCluster"));
    cluster_cmd_req_header_t *rsp = (cluster_cmd_req_header_t*)(rx.getFrameData() + rx.getDataOffset());
    if (isDebugXbeeEnabled) printHex(Serial, rsp->frame_control);
    if (isDebugXbeeEnabled) Serial.println();
    if (isDebugXbeeEnabled) printHex(Serial, rsp->sequence_number);
    if (isDebugXbeeEnabled) Serial.println();
    if (isDebugXbeeEnabled) printHex(Serial, rsp->command);
    if (isDebugXbeeEnabled) Serial.println();
    
    if(rsp->command == 0x00){
      if (isDebugXbeeEnabled) Serial.println(F("Read attributes"));
      uint8_t *attributes = ((uint8_t*)rsp + sizeof(cluster_cmd_req_header_t));
      uint16_t attribute = (uint16_t)attributes[1] << 8 | attributes[0];
      
      if (isDebugXbeeEnabled) Serial.print(F("Reading 0x"));
      if (isDebugXbeeEnabled) printHex(Serial, attributes[0]);
      if (isDebugXbeeEnabled) Serial.println();
      if (isDebugXbeeEnabled) printHex(Serial, attributes[1]);
      if (isDebugXbeeEnabled) Serial.println();
      
      if (isDebugXbeeEnabled) Serial.print(F("Reading 0x"));
      if (isDebugXbeeEnabled) printHex(Serial, attribute);
      if (isDebugXbeeEnabled) Serial.println();

      if(attribute == 0x0000){
        if (isDebugXbeeEnabled) Serial.println(F("Reading ZCL_VERSION"));
      }
      if(attribute == 0x0001){
        if (isDebugXbeeEnabled) Serial.println(F("Reading APPLICATION_VERSION"));
      }
      uint8_t payload[] = {0x00, rsp->sequence_number, 0x01, attributes[0], attributes[1], 0x00, 0x20, 0x03};
      
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = rx.getClusterId();
      uint16_t profileId = rx.getProfileId();
      
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      xbee.send(tx);
    }
  }
  if (rx.getClusterId() == 0x0402){
    if (isDebugXbeeEnabled) Serial.println(F("TemperatureMeasurementCluster"));
    cluster_cmd_req_header_t *rsp = (cluster_cmd_req_header_t*)(rx.getFrameData() + rx.getDataOffset());
    if(rsp->command == 0x00){
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = rx.getClusterId();
      uint16_t profileId = rx.getProfileId();
      
      uint8_t *attributes = ((uint8_t*)rsp + sizeof(cluster_cmd_req_header_t));
      uint16_t attribute = (uint16_t)attributes[1] << 8 | attributes[0];
      
      if(attribute == 0x0000){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MeasuredValue"));
      }
      if(attribute == 0x0001){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MinMeasuredValue"));
      }
      if(attribute == 0x0002){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MaxMeasuredValue"));
      }
      if(attribute == 0x0003){
        if (isDebugXbeeEnabled) Serial.println(F("Reading Tolerance"));
      }
      uint8_t payload[] = {0x00, rsp->sequence_number, 0x01, attributes[0], attributes[1], 0x00, 0x20, (uint8_t)temperature};
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      xbee.send(tx);
    }
  }
  if (rx.getClusterId() == 0x0405){
    if (isDebugXbeeEnabled) Serial.println(F("RelativeHumidityMeasurementCluster"));
    cluster_cmd_req_header_t *rsp = (cluster_cmd_req_header_t*)(rx.getFrameData() + rx.getDataOffset());
    if(rsp->command == 0x00){
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = rx.getClusterId();
      uint16_t profileId = rx.getProfileId();
      
      uint8_t *attributes = ((uint8_t*)rsp + sizeof(cluster_cmd_req_header_t));
      uint16_t attribute = (uint16_t)attributes[1] << 8 | attributes[0];
      
      if(attribute == 0x0000){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MeasuredValue"));
      }
      if(attribute == 0x0001){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MinMeasuredValue"));
      }
      if(attribute == 0x0002){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MaxMeasuredValue"));
      }
      if(attribute == 0x0003){
        if (isDebugXbeeEnabled) Serial.println(F("Reading Tolerance"));
      }
      uint8_t payload[] = {0x00, rsp->sequence_number, 0x01, attributes[0], attributes[1], 0x00, 0x20, (uint8_t)humidity};
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      xbee.send(tx);
    }
  }
  if (rx.getClusterId() == 0x0B04){
    if (isDebugXbeeEnabled) Serial.println(F("ElectricalMeasurementCluster"));
    cluster_cmd_req_header_t *rsp = (cluster_cmd_req_header_t*)(rx.getFrameData() + rx.getDataOffset());
    if(rsp->command == 0x00){
      XBeeAddress64 &addr64 = rx.getRemoteAddress64();
      uint16_t addr16 = rx.getRemoteAddress16();
      uint8_t broadcastRadius = 0;
      uint8_t option = 0;
      uint8_t frameId = xbee.getNextFrameId();
      uint8_t srcEndpoint = rx.getDstEndpoint();
      uint8_t dstEndpoint = rx.getSrcEndpoint();
      uint16_t clusterId = rx.getClusterId();
      uint16_t profileId = rx.getProfileId();
      
      uint8_t *attributes = ((uint8_t*)rsp + sizeof(cluster_cmd_req_header_t));
      uint16_t attribute = (uint16_t)attributes[1] << 8 | attributes[0];
      
      if(attribute == 0x0000){
        if (isDebugXbeeEnabled) Serial.println(F("Reading MeasurementType"));
      }
      uint8_t payload[] = {0x00, rsp->sequence_number, 0x01, attributes[0], attributes[1], 0x00, 0x20, (uint8_t)moisture_dat};
      ZBExplicitTxRequest tx(addr64,addr16, broadcastRadius, option, payload, sizeof(payload), frameId, srcEndpoint, dstEndpoint, clusterId, profileId);
      xbee.send(tx);
    }
  }
}

void setup() {
  flower.Initialization();//Initialization for the watering kit
  
  if (isDebugEnabled || isDebugXbeeEnabled) Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  if (isDebugEnabled || isDebugXbeeEnabled) Serial.println("Goodnight moon!");

  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  // Make sure that any errors are logged to Serial. The address of
  // Serial is first cast to Print*, since that's what the callback
  // expects, and then to uintptr_t to fit it inside the data parameter.
  if (isDebugXbeeEnabled) xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&Serial);
  if (isDebugXbeeEnabled) xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);
  if (isDebugXbeeEnabled) xbee.onZBTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);

  // These are called when an actual packet received
  xbee.onZBRxResponse(zbReceive);
  xbee.onRx16Response(receive16);
  xbee.onRx64Response(receive64);
  xbee.onZBExplicitRxResponse(receiveZBExplicitRx);

  // Print any unhandled response with proper formatting
  if (isDebugXbeeEnabled) xbee.onOtherResponse(printResponseCb, (uintptr_t)(Print*)&Serial);

  // Enable this to print the raw bytes for _all_ responses before they
  // are handled
  if (isDebugXbeeEnabled) xbee.onResponse(printRawResponseCb, (uintptr_t)(Print*)&Serial);

  // setup timed actions 
  t.every(1L*60L*1000L, queueManager);// double check queue to see if there is work to do
//  t.every(10L*60L*1000L, timeToUpdate); //send update to smartThings hub every 10 min
  t.every(1L*60L*1000L, timeToGetReads); //
  t.every(1L*60L*1000L, timeToCheckEnv); //

}

void timeToCheckEnv() {
  if (isDebugEnabled) Serial.println(F("timeToCheckEnv"));
  scheduleCheckEnv();
}
void scheduleCheckEnv() {
  if (isDebugEnabled) Serial.println(F("scheduleCheckEnv"));
  doCheckEnv = true; 
}
void CheckEnv() {
  if (isDebugEnabled) Serial.println(F("CheckEnv"));
  int humidity_max;
  humidity_max = flower.ADJ_humMax(); 
  if(existReads && trafficCop==0 && moisture_dat<=humidity_max && temperature<=MaxTemprature){
    if (isDebugEnabled) Serial.println(F("enqueue"));
    int addStation=1;
    queue[addStation]=1;
    stationTime[addStation]=4L*1000L;
  }
}

void timeToUpdate() {
  if (isDebugEnabled) Serial.println(F("timeToUpdate"));
//  scheduleUpdate();
//  schedulePumpUpdate();
}
void timeToGetReads() {
  if (isDebugEnabled) Serial.println(F("timeToGetReads"));
//  scheduleUpdate();
  scheduleGetReads();
}
void scheduleUpdate() {
  if (isDebugEnabled) Serial.println(F("scheduleUpdate"));
  doUpdate = true; 
}
void scheduleGetReads() {
  if (isDebugEnabled) Serial.println(F("scheduleGetReads"));
  doGetReads = true;
}

void GetReads() {
  if (isDebugEnabled) Serial.println(F("GetReads"));
  temperature = flower.getTemperature();
  moisture_dat = flower.MoistureSensor();
  humidity = flower.getHumidity();
  Serial.print("Temperature is :");
  Serial.println(temperature);
  Serial.print("Soil Moisture is :");
  Serial.println(moisture_dat);
  Serial.print("Humidity is :");
  Serial.println(humidity);
  existReads = true;
}

void schedulePumpUpdate() {
  doPumpUpdate = true;
}
void sendPumpUpdate() {
  if (isDebugEnabled) Serial.println(F("sendPumpUpdate"));
}

void queueManager() 
{
  Serial.print("trafficCop is :");
  Serial.println(trafficCop);
  Serial.print("queue[1] is :");
  Serial.println(queue[1]);
  Serial.print("stationTimer[1] is :");
  Serial.println(stationTimer[1]);
  Serial.print("stationTime[1] is :");
  Serial.println(stationTime[1]);
  
  if (isDebugEnabled) Serial.println(F("queueManager"));
  int i=1;
  while (i<2) {
    if (trafficCop==0 && queue[i]==1) {
      //ready for next in line 
      trafficCop=i;
      scheduleToggleOn();
    }
    i++;
  }
}
void scheduleToggleOn() {
  if (isDebugEnabled) Serial.println(F("scheduleToggleOn"));
  doToggleOn = true;
}
void toggleOn() {
  if (isDebugEnabled) Serial.println(F("toggleOn"));
  queue[trafficCop]=2;

  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  digitalWrite(7,LOW);
  digitalWrite(4,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);

  Serial.print("stationTimer is :");
  Serial.println(stationTimer[trafficCop]);
  Serial.print("stationTime is :");
  Serial.println(stationTime[trafficCop]);
  if (stationTimer[trafficCop]>0)
    t.stop(stationTimer[trafficCop]); // Kill any previously started timers.
  Serial.print("stationTimer is :");
  Serial.println(stationTimer[trafficCop]);
  Serial.print("stationTime is :");
  Serial.println(stationTime[trafficCop]);
  stationTimer[trafficCop] = t.after (stationTime[trafficCop], toggleOff);
  Serial.print("stationTimer is :");
  Serial.println(stationTimer[trafficCop]);
  Serial.print("stationTime is :");
  Serial.println(stationTime[trafficCop]);
}
void toggleOff() {
  if (isDebugEnabled) Serial.println(F("toggleOff"));
  flower.pumpOff();
  queue[trafficCop]=0; //remove relay from from queue 
  trafficCop=0; //ready to check queue or watch for new commmands
}

void loop() {
  //run timer 
  t.update();

  if (doToggleOn){
    toggleOn();
    doToggleOn=false;
  }

  if (doCheckEnv) {
    CheckEnv();
    doCheckEnv = false;
  }

  if (doPumpUpdate) {
    sendPumpUpdate();
    doPumpUpdate = false;
  }
  if (doGetReads) {
    GetReads();
    doGetReads = false;
  }
  
  // Continuously let xbee read packets and call callbacks.
  xbee.loop();
}

