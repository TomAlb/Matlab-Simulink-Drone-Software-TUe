/*
 * NatNetSDK.h - Library for converting received NatNetSDK data to a easy to use data struct.
 * Created by Tom Albers, March 289, 2019.
 * Released to support Pixhawk drone flight on TU/e
 */

 #pragma once
 #include "NatNetSDK.h"
 

 void UnpackMessage(byte *incomingPacket){
  //Serial.println("Hello World!");
  sFrameOfMocapData* MSG;
  
  // Raw data transformation
  MSG->
    int MSGID = (unsigned)((incomingPacket[1] << 8) | incomingPacket[0]);
    int  ByteCount = (unsigned)((incomingPacket[3] << 8) | incomingPacket[2]);
    long int  FrameNR = (unsigned long)((incomingPacket[8] << 36) | (incomingPacket[7] << 24) | (incomingPacket[6] << 16) | (incomingPacket[5] << 8) | incomingPacket[4]);
    long int  MarkerSetCount = (unsigned long)((incomingPacket[15] << 36) | (incomingPacket[14] << 24) | (incomingPacket[13] << 36) | (incomingPacket[12] << 24) | (incomingPacket[11] << 16) | (incomingPacket[10] << 8) | incomingPacket[9]);
    int  RigidBodyCount = (unsigned long)((incomingPacket[19] << 24) | (incomingPacket[18] << 16) | (incomingPacket[17] << 8) | incomingPacket[16]);
    long int  ID = (signed long)((incomingPacket[23] << 24) | (incomingPacket[22] << 16) | (incomingPacket[21] << 8) | incomingPacket[20]);
    float  pos[] = {(signed long)((incomingPacket[27] << 24) | (incomingPacket[26] << 16) | (incomingPacket[25] << 8) | incomingPacket[24]), (unsigned long)((incomingPacket[31] << 24) | (incomingPacket[30] << 16) | (incomingPacket[29] << 8) | incomingPacket[28]), (unsigned long)((incomingPacket[35] << 24) | (incomingPacket[34] << 16) | (incomingPacket[33] << 8) | incomingPacket[32])};
    float  quat[] = {(signed long)((incomingPacket[39] << 24) | (incomingPacket[38] << 16) | (incomingPacket[37] << 8) | incomingPacket[36]), (unsigned long)((incomingPacket[43] << 24) | (incomingPacket[42] << 16) | (incomingPacket[41] << 8) | incomingPacket[40]), (unsigned long)((incomingPacket[47] << 24) | (incomingPacket[46] << 16) | (incomingPacket[45] << 8) | incomingPacket[44]), (unsigned long)((incomingPacket[51] << 24) | (incomingPacket[50] << 16) | (incomingPacket[49] << 8) | incomingPacket[48])};


    //Reconstruct messages
    //Reconstruct messages
    Serial.println("------------");
    Serial.println("Begin Packet");
    Serial.println("------------");
    Serial.printf("Message ID : %d\n", MSGID);
    Serial.printf("Byte count : %d\n", ByteCount);
    Serial.printf("Frame # : %d\n", FrameNR);
    Serial.printf("Marker Set Count : %d\n", MarkerSetCount);
    Serial.printf("Rigid Body Count : %d\n", RigidBodyCount);
    Serial.printf("ID : %d\n", ID);
    Serial.printf("pos : %f\t%f\t%f\n", pos[0], pos[1], pos[2]);
    Serial.printf("quat : %f\t%f\t%f\t%f\n", quat[0], quat[1], quat[2], quat[3]);
    Serial.println("------------");
    Serial.println("End Packet");
    Serial.println("------------");
 }
