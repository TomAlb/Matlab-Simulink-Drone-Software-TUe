#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include "NatNetSDK2.h"

/*******************************************************************************************************************************
   SET LOOP TIMMING PARAMETERS
 *******************************************************************************************************************************/
#define STD_LOOP_TIME 1000   // Fixed time loop of 10 milliseconds ~ 100Hz
unsigned long loopStartTime;
unsigned long loopStopTime;
unsigned int Loop = 0;

/*******************************************************************************************************************************
   SET WIFI COMMUNICATION SETTINGS
 *******************************************************************************************************************************/
const char* ssid = "";               // your network SSID (name)
const char* pass = "";               // your network password

WiFiUDP Udp;                                  // A UDP instance to let us send and receive packets over UDP
IPAddress ipMulti(239, 255, 42, 99);          // Multicast declarations
//IPAddress ipMulti(192, 168, 1, 21);         // Multicast declarations

/*******************************************************************************************************************************
   SET OPTITRACK RELATED SETTINGS
 *******************************************************************************************************************************/
#define BUFFER_LENGTH 566                     // Buffer length, is the length of the expected received message
byte incomingPacket[BUFFER_LENGTH];           // Incomming packet
UDP_Packet_t MSG;                             // Message declaration
String Optitrack_Message = "$OPTI,";          // Base Optitrack Message
int cnt = 0;

/*******************************************************************************************************************************
   SETUP LOOP
   - Setup serial communication
   - List available networks
   - Setup WiFi communication
   - Send WiFi connection information over serial
   - Start multicast connection
 *******************************************************************************************************************************/
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println();

  delay(1000);

  listNetworks();

  Serial.printf("\nAttempting to connect to SSID: %s\n", ssid);
  // setting up Station AP
  WiFi.begin(ssid, pass);

  // attempt to connect to WiFi network:
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Succesfully connected to WiFi!");
  printWifiStatus();

  //Start Multicast connection
  Udp.beginMulticast(WiFi.localIP(), ipMulti, NATNET_DEFAULT_PORT_DATA);
}

/*******************************************************************************************************************************
   MAIN LOOP
 *******************************************************************************************************************************/
void loop() {
  // Get timming information on loop cycles
  loopStartTime = micros();


  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    int len = Udp.read(incomingPacket, BUFFER_LENGTH);
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    if (len > 0) {
      incomingPacket[len] = 0;
      for (int k = 0; k < len; k++) {
        MSG.UDPPacket[k] = incomingPacket[k];
      }
    }

    /*
      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());

      Serial.println("---------------------------------------------------------------");
      Serial.println("Begin Packet");
      Serial.println("---------------------------------------------------------------");

      Serial.printf("Message ID             : %d\n", MSG.Frame.msgID);
      Serial.printf("Byte count             : %d\n", MSG.Frame.ByteCount);
      Serial.printf("Frame #                : %d\n", MSG.Frame.MocapData.FrameNr);
      Serial.printf("Marker Set Count       : %d\n", MSG.Frame.MocapData.nMarkerSets);
      Serial.printf("Other Marker Set Count : %d\n", MSG.Frame.MocapData.nOtherMarkers);
      Serial.printf("Rigid Body Count       : %d\n", MSG.Frame.MocapData.nRigidBodies);

      for(int k=0; k<MSG.Frame.MocapData.nRigidBodies; k++)
      {
      Serial.printf("Rigid Body ID          : %d\n", MSG.Frame.MocapData.RigidBodies[k].ID);
      Serial.printf("Position               : |%0.3f %0.3f %0.3f|\n", MSG.Frame.MocapData.RigidBodies[k].x, MSG.Frame.MocapData.RigidBodies[k].y, MSG.Frame.MocapData.RigidBodies[k].z);
      Serial.printf("Orientation            : |%0.3f %0.3f %0.3f %0.3f|\n", MSG.Frame.MocapData.RigidBodies[k].qx, MSG.Frame.MocapData.RigidBodies[k].qy, MSG.Frame.MocapData.RigidBodies[k].qz, MSG.Frame.MocapData.RigidBodies[k].qw);
      //Serial.printf("Mean Error             : %0.3f\n", MSG.Frame.MocapData.RigidBodies[k].MeanError);
      //Serial.printf("Parameters             : %d\n", MSG.Frame.MocapData.RigidBodies[k].params);
      }
      //Serial.printf("Rigid Body ID          : %d\n", MSG.Frame.MocapData.RigidBodies[1].ID);
      Serial.println("---------------------------------------------------------------");
      Serial.println("End Packet");
      Serial.println("---------------------------------------------------------------");
    */

    Optitrack_Message = "$OPTI,";          // Base Optitrack Message
    //Optitrack_Message = String(Optitrack_Message + MSG.Frame.MocapData.FrameNr + ",");
    Optitrack_Message = String(Optitrack_Message + cnt + ",");
    cnt = cnt + 1;
    Optitrack_Message = String(Optitrack_Message + MSG.Frame.MocapData.RigidBodies[0].ID + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].x, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].y, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].z, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].qx, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].qy, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].qz, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].qw, 3) + ",");
    Optitrack_Message = String(Optitrack_Message + String(MSG.Frame.MocapData.RigidBodies[0].MeanError, 3));
    
    // CRC checksum
    byte CRC = CalculateCRC(Optitrack_Message);
    Optitrack_Message = String(Optitrack_Message + "*" + String(CRC, HEX));

    Serial.println(Optitrack_Message);
    //Serial.println(Optitrack_Message.length());
    /*
      Serial.printf("$OPTI");
      Serial.printf(",%d", MSG.Frame.MocapData.FrameNr);
      Serial.printf(",%d", MSG.Frame.MocapData.RigidBodies[0].ID);
      Serial.printf(",%0.3f,%0.3f,%0.3f", MSG.Frame.MocapData.RigidBodies[0].x, MSG.Frame.MocapData.RigidBodies[0].y, MSG.Frame.MocapData.RigidBodies[0].z);
      Serial.printf(",%0.3f,%0.3f,%0.3f", MSG.Frame.MocapData.RigidBodies[0].qx, MSG.Frame.MocapData.RigidBodies[0].qy, MSG.Frame.MocapData.RigidBodies[0].qz, MSG.Frame.MocapData.RigidBodies[0].qw);
      Serial.printf(",%0.3f", MSG.Frame.MocapData.RigidBodies[0].MeanError);
      Serial.printf("*\n");

      //Serial.printf("Loop#: %d - %d - %d\n", Loop, micros() - loopStartTime, MsgNr++);
    */

  }


  // Delay loop until STD_LOOP_TIME is met
  Loop++;
  loopStopTime = micros();
  if (loopStopTime - loopStartTime < STD_LOOP_TIME) {
    delayMicroseconds(STD_LOOP_TIME - (micros() - loopStartTime));
  }
}


/*******************************************************************************************************************************
   LIST ALL AVAILABLE NETWORKS
 *******************************************************************************************************************************/
void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks Started  **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  Serial.print("number of available networks: ");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
  }
  Serial.println("** Scan Networks Finished **");
}

/*******************************************************************************************************************************
   PRINT ENCRYPTION TYPE
 *******************************************************************************************************************************/
void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
    default:
      Serial.println("Unknown");
      break;
  }
}

/*******************************************************************************************************************************
   PRINT WIFI STATUS
 *******************************************************************************************************************************/
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID:\t\t\t");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  Serial.print("IP Address:\t\t");
  Serial.println(WiFi.localIP());

  Serial.printf("Netmask:\t\t");
  Serial.println(WiFi.subnetMask());

  Serial.printf("Gateway:\t\t");
  Serial.println(WiFi.gatewayIP());
  
  Serial.printf("Broadcast:\t\t");
  Serial.println(WiFi.gatewayIP());
  

  // print the received signal strength:
  Serial.printf("Signal strength (RSSI):\t");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

/*******************************************************************************************************************************
   CALCULATE CRC CHECKSUM
 *******************************************************************************************************************************/
byte CalculateCRC(String MSG) {
  byte CRC = 0;
  for (int i = 0; i < MSG.length(); i++) {
    CRC ^= byte(MSG[i]);
    //Serial.print(CRC, HEX);
    //Serial.print("|");
  }  
  //Serial.println();
  return CRC;
}
