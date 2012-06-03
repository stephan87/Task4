#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

// define SIMULATION if you want to use Tossim, otherwise don't
#define SIMULATION2
#ifdef SIMULATION
#define GETTIME time(NULL)
#define GETRSSI (55) / (msgReceived->hops + 1)
#else
#define GETTIME call LocalTime.get()
#define GETRSSI call CC2420Packet.getRssi(msg)
#endif


enum {
  AM_COMMANDMSG 			= 6,		// channel identifier for commands
  AM_BEACONMSG 				= 5,		// channel identifier for beacons
  AM_SENSORMSG				= 4,		// channel identifier for sensormsgs
  AM_TABLEMSG				= 3,		// channel identifier for tablemsgs
  AM_BEACONINTERVAL 		= 1000,		// period in which beacon msgs are sent
  AM_BEACONTIMEOUT			= 8, 		// in seconds
  AM_TABLESIZE 				= 4,		// maximum amount of table entries
  AM_ACKTIMEOUT				= 2000,		// timeout in ms within the acks must be received
  AM_MAXNODEID				= 65535,	// used as default undefined value
  NREADINGS 				= 5, 		// count of sensor reading until transmission
  DEFAULT_SAMPLING_INTERVAL = 1000, 	// Default sensor read interval period.
  SERIAL_ADDR				= 99, 		// serial address
  UNDEFINED					= 0xFFFF,
  ACK_RETRANSMIT_TIMEOUT	= 1000,
  READLOG_INTERVAL 			= 25,		// read from log every milliseconds 
  CONFIG_ADDR 				= 0,
  CONFIG_STATE_INIT			= 0,
  CONFIG_STATE_WRITING  	= 1,
  CONFIG_STATE_READING  	= 2,
  SENSOR_HUMIDITY			= 1, 		
  SENSOR_TEMPERATURE		= 2, 
  SENSOR_LIGHT				= 3 	
};

typedef nx_struct CommandMsg {
  nx_uint16_t seqNum;
  nx_uint16_t ledNum;
  nx_uint16_t sender;
  nx_uint16_t receiver;
  nx_uint8_t reqSensor; 	// requests sending of sensor data 0 = no, 1 = yes
  nx_uint8_t isAck;
} CommandMsg;

typedef nx_struct BeaconMsg {
  nx_uint16_t sender;
  nx_uint16_t parent;		// chosen parent
  nx_uint16_t hops;			// hop count to mote 0
  nx_uint16_t avgRSSI;		// average RSSI
  nx_uint16_t version;		// avoid loops, update if connection is lost and then broadcast updated version
} BeaconMsg;

typedef struct MoteTableEntry {
  uint16_t nodeId;
  bool ackReceived;
  uint16_t lastContact;
  bool expired;
  uint16_t hops;			// hop count to mote 0
  uint16_t avgRSSI;			// average RSSI
  uint16_t parentMote;		// chosen parent mote of neighbor
  bool childMote;			// child of current mote; forward msg only if child == true
} MoteTableEntry;

typedef nx_struct SensorMsg {
  nx_uint16_t receiver; 			/* should be serial */
  nx_uint16_t sender; 				/* Mote id of sending mote. */
  nx_uint8_t  sensor; 				/* From which sensor? 0 means not active*/
  nx_uint16_t version; 				/* Version of the interval. */
  nx_uint16_t readings[NREADINGS];  /* data */
} SensorMsg;

typedef nx_struct TableMsg {
  nx_uint16_t seqNum;
  nx_uint16_t sender; 
  nx_uint8_t  receiver; 
  nx_uint16_t nodeId[AM_TABLESIZE];
  nx_uint16_t lastContact[AM_TABLESIZE]; 
  nx_uint16_t parent; 		// chosen parent mote of sender
} TableMsg;

typedef struct QueueInfo{
	uint8_t type;
	uint32_t timestamp; 
} QueueInfo;

typedef struct config_t {
	uint8_t  state;        // state of the storage: no data/init, writing, reading  	
	uint16_t version;
} config_t;


typedef nx_struct logentry_t {
  nx_uint8_t sensor;					/* From which sensor */
  nx_uint16_t readings[NREADINGS];		/* Data of a sensor  */
  nx_uint16_t version; 				    /* Version of the interval. */
} logentry_t;


#endif
