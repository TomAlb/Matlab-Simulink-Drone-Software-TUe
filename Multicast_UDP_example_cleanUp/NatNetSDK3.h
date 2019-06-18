/*
 * NatNetSDK.h - Library for converting received NatNetSDK data to a easy to use data struct.
 * Created by Tom Albers, March 289, 2019.
 * Released to support Pixhawk drone flight on TU/e
 */
#pragma once

#include <stdint.h>


#if !defined( NULL )
#   include <stddef.h>
#endif

/*
#define NATNET_CALLCONV
#define NATNET_DEPRECATED( msg )     __attribute__((deprecated(msg)))


// storage class specifier
// - to link to NatNet dynamically, define NATNETLIB_IMPORTS and link to the NatNet import library.
// - to link to NatNet statically, link to the NatNet static library.

#if defined( NATNETLIB_EXPORTS )
#    define NATNET_API               __attribute((visibility("default")))
#elif defined( NATNETLIB_IMPORTS )
#    define NATNET_API
#else
#    define NATNET_API
#endif


//Client/Server Configurations
#define NATNET_DEFAULT_PORT_COMMAND 1510
*/
#define NATNET_DEFAULT_PORT_DATA    1511
/*
#define NATNET_DEFAULT_MULTICAST_ADDRESS  "239.255.42.99" //IANA, local network


// Model Limits
#define MAX_MODELS                2000    // maximum number of total models (data descriptions)
#define MAX_MARKERSETS            1000    // maximum number of MarkerSets
#define MAX_RIGIDBODIES           1000    // maximum number of RigidBodies
#define MAX_NAMELENGTH            256     // maximum length for strings
#define MAX_MARKERS               200     // maximum number of markers per MarkerSet
#define MAX_RBMARKERS             20      // maximum number of markers per RigidBody
#define MAX_SKELETONS             100     // maximum number of Skeletons
#define MAX_SKELRIGIDBODIES       200     // maximum number of RigidBodies per Skeleton
#define MAX_LABELED_MARKERS       1000    // maximum number of labeled markers per frame
#define MAX_UNLABELED_MARKERS     1000    // maximum number of unlabeled (other) markers per frame
#define MAX_FORCEPLATES           8       // maximum number of force plates
#define MAX_DEVICES               32      // maximum number of peripheral devices
#define MAX_ANALOG_CHANNELS       32      // maximum number of data channels (signals) per analog/force plate device
#define MAX_ANALOG_SUBFRAMES      30      // maximum number of analog/force plate frames per mocap frame
#define MAX_PACKETSIZE            100000  // max size of packet (actual packet size is dynamic)

// Client/Server message ids
#define NAT_CONNECT               0
#define NAT_SERVERINFO            1
#define NAT_REQUEST               2
#define NAT_RESPONSE              3
#define NAT_REQUEST_MODELDEF      4
#define NAT_MODELDEF              5
#define NAT_REQUEST_FRAMEOFDATA   6
#define NAT_FRAMEOFDATA           7
#define NAT_MESSAGESTRING         8
#define NAT_DISCONNECT            9
#define NAT_KEEPALIVE             10
#define NAT_DISCONNECTBYTIMEOUT   11
#define NAT_ECHOREQUEST           12
#define NAT_ECHORESPONSE          13
#define NAT_DISCOVERY             14
#define NAT_UNRECOGNIZED_REQUEST  100

#define UNDEFINED                 999999.9999

typedef float MarkerData[3];                // posX, posY, posZ

// MarkerSet Data (single frame of one MarkerSet)
typedef struct sMarkerSetData
{
//    char szName[MAX_NAMELENGTH];            // MarkerSet name
    int32_t nMarkers;                       // # of markers in MarkerSet
    MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
};

// Rigid Body Data (single frame of one rigid body)
typedef struct sRigidBodyData
{
    int32_t ID;                             // RigidBody identifier: 
                                            // For rigid body assets, this is the Streaming ID value. 
                                            // For skeleton assets, this combines both skeleton ID (High-bit) and Bone ID (Low-bit).

    float x, y, z;                          // Position
    float qx, qy, qz, qw;                   // Orientation
    float MeanError;                        // Mean measure-to-solve deviation
    int16_t params;                         // Host defined tracking flags
/*
#if defined(__cplusplus)
    sRigidBodyData()
        : ID( 0 )
        , params( 0 )
    {
    }
#endif

};

// Single frame of data  (for all tracked objects)
typedef struct sFrameOfMocapData
{
  int32_t FrameNr;                          // host defined frame number

  int32_t nMarkerSets;                      // # of marker sets in this frame of data
  sMarkerSetData MocapData[MAX_MARKERSETS]; // MarkerSet data

  int32_t nOtherMarkers;                          // # of undefined markers
//  MarkerData* OtherMarkers;                       // undefined marker data

  int32_t nRigidBodies;                           // # of rigid bodies
  sRigidBodyData RigidBodies[MAX_RIGIDBODIES];    // Rigid body data
  
};
*/
typedef struct message_t {
  uint16_t msgID;
  uint16_t ByteCount;
  int32_t FrameNR;
};

typedef union UDP_Packet_t{
  message_t Frame;
  byte UDPPacket[sizeof(message_t)];
};

typedef union nMarkerSets_t{
  int32_t nMarkerSets;
  byte UDPPacket[sizeof(int32_t)];
};

typedef union nOtherMarkers_t{
  int32_t nOtherMarkers;
  byte UDPPacket[sizeof(int32_t)];
};

typedef union nRigidBodies_t{
  int32_t nRigidBodies;
  byte UDPPacket[sizeof(int32_t)];
};

typedef union Markers_t{
  char SzName[8];
  int32_t nMarkers;
  byte UDPPacket[12];
};

// Rigid Body Data (single frame of one rigid body)
typedef struct sRigidBodyData {
    int32_t ID;                             // RigidBody identifier: 
                                            // For rigid body assets, this is the Streaming ID value. 
                                            // For skeleton assets, this combines both skeleton ID (High-bit) and Bone ID (Low-bit).
    float x, y, z;                          // Position
    float qx, qy, qz, qw;                   // Orientation
    float MeanError;                        // Mean measure-to-solve deviation
    int16_t params;                         // Host defined tracking flags
    
    byte UDPPacket[38];
};
