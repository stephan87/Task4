#include "TestSerial.h"
#include "StorageVolumes.h"
#include "printf.h"


module TestSerialC @safe()
{
	uses {
	    interface Boot;
	    interface SplitControl as SerialControl;
	    interface SplitControl as RadioControl;
	
	    interface AMSend as SerialSend[am_id_t id];
	    interface Receive as SerialReceive[am_id_t id];
	    interface Packet as SerialPacket;
	    interface AMPacket as SerialAMPacket;
	    
	    interface AMSend as RadioSend[am_id_t id];
	    interface Receive as RadioReceive[am_id_t id];
	    interface Packet as RadioPacket;
	    interface AMPacket as RadioAMPacket;
	
		interface Timer<TMilli> as BeaconTimer;
		interface Timer<TMilli> as AckTimer;
		interface Timer<TMilli> as SensorTimer;
		interface Timer<TMilli> as ReadLogTimer;
		interface Timer<TMilli> as TableTimer;
		
		interface LogRead;
    	interface LogWrite;
	    
#ifndef SIMULATION
	    interface LocalTime<TSecond>;
	    interface CC2420Packet;
#endif
	    interface Leds;
	    
	    interface Read<uint16_t> as SensorHumidity;
	    interface Read<uint16_t> as SensorTemperature;
	    interface Read<uint16_t> as SensorLight;
	    
	    interface Queue<message_t *> as RadioQueue;
	    interface Pool<message_t> as RadioMsgPool;
	    interface Queue<message_t *> as SerialQueue;
	    interface Pool<message_t> as SerialMsgPool;
	    interface Queue<uint8_t> as SerialTypeQueue;
	    interface Queue<QueueInfo> as RadioTypeQueue;
	    
	    interface Random;
	    interface PacketAcknowledgements;
	    interface ActiveMessageAddress;
	    
	    // storage interfaces
	    interface ConfigStorage as Config;
    	interface Mount;
  	}
}
implementation
{

	uint16_t localSeqNumberCommand = 0; ///< stores the msg sequence number
	uint16_t localSeqNumberSensor = 0;
	uint16_t localSeqNumberTable = 0;

	
	uint16_t tableSendCounter = 0;
	bool radioBusy	= FALSE;
	bool serialBusy	= FALSE;
	bool nodeBusy 	= FALSE;
	
	message_t sndRadioLast;		///< stores the last sent radio message -  used for retransmit
	message_t sndSerialLast;	///< stores the last sent serial message - used for retransmit
	message_t rcvRadio;			///< stores the last received radio msg
	message_t rcvSerial;		///< stores the last received serial msg
	
	MoteTableEntry neighborTable[AM_TABLESIZE];		///< stores the neighbors of the node
		
	SensorMsg collectorHumidityMsg; 		// collects sensor data to send ... sensor = 1
	SensorMsg collectorTemperatureMsg; 	// collects sensor data to send ... sensor = 2
	SensorMsg collectorLightMsg; 			// collects sensor data to send ... sensor = 3
	
	// counts the sensor readings
	uint8_t readingCountSensorHumidity;
	uint8_t readingCountSensorTemperature;
	uint8_t readingCountSensorLight;
	
	// initialization methods
	void initNeighborTable();
	
	// routing
	uint16_t localHops 		= UNDEFINED; 	// lower is better
	uint16_t localAvgLqi 	= 0;			// higher is better
	uint16_t chosenParent 	= UNDEFINED; 	// UNDEFINED = no parent 
	uint16_t localVersion 	= UNDEFINED; 	// not set
	
	// methods which capsulate the sending of messages
  	void radioSend(CommandMsg* msgToSend);
  	void enqBeacon();
  	void serialReflect();
  	void forwardTable(TableMsg* msg, QueueInfo info); 
  	void sendRadioAck();
  	void enqTable();
  	void radioSendSensorMsg(SensorMsg *msg, QueueInfo info);
  	void serialSendSensorMsg(SensorMsg *msg);
  	void serialSendTable(TableMsg* msg);
  	
  	// methods which use sensors 
  	void startSensorTimer();
  	void initSensors();
  	void activateSensor(uint8_t sensor);
  	void deactivateSensor(uint8_t sensor);
  	
  	task void sendQueueTask();
  	task void serialSendQueueTask();
  	void enqAck();
  	message_t sendbuf;
  	int seqNums[10];
  	int seqSensors[10];
  	bool checkForAck = FALSE;
	
	// other methods
  	void startTableTimer();
  	
  	// storage
  	config_t conf;
  	logentry_t logEntry;
  	bool storageBusy = FALSE;		// should be set to true if writting starts
  	
  	
  	
  	
  	
  	/*******************************************************************************************
  	*
  	*								Boot, initTable
  	*
  	*******************************************************************************************/
  	
  	/*
  	*	Method which is called after the Mote booted
  	*	-initializes sensors and neighbor table
  	*	-start periodic sending of beacon messages
  	*/
  	event void Boot.booted() 
  	{	
    	call RadioControl.start();
    	call ActiveMessageAddress.setAddress(6,TOS_NODE_ID);
    	
    	// init conf 
    	if (call Mount.mount() != SUCCESS)
    	{
    	     dbg("Storage","Storage mount failed\n");
	    }
	    
		    
    	if(TOS_NODE_ID == 0)
    	{
    		call SerialControl.start();
    		// routing... new version
    		localVersion = call Random.rand16(); 
    		localHops = 0; 	// lower is better
			localAvgLqi = 255;		// higher is better
			chosenParent = 0; // UNDEFINED = no parent 
   		}
    	
    	initNeighborTable();
    	startTableTimer();
    	initSensors();
    	
    	dbg("TestSerialC","message length: SensorMsg =%d\n",sizeof(SensorMsg));
    	dbg("TestSerialC","message length: CommandMsg = %d\n",sizeof(CommandMsg));
    	dbg("TestSerialC","message length: TableMsg = %d\n",sizeof(TableMsg));
    	dbg("TestSerialC","message length: BeaconMsg = %d\n",sizeof(BeaconMsg));
    	dbg("TestSerialC","message length: message_t = %d\n",sizeof(message_t));
    	
    	
    	call BeaconTimer.startPeriodic( AM_BEACONINTERVAL );
  	}
  	
  	
  	/*
  	*	Initializes the neighbor table with default values:
  	*/
  	void initNeighborTable()
  	{
  		int i;
  		for(i = 0;i<AM_TABLESIZE;i++)
  		{
  			neighborTable[i].nodeId 		= AM_MAXNODEID;
  			neighborTable[i].ackReceived 	= FALSE;
			neighborTable[i].lastContact	= 0;
		  	neighborTable[i].expired		= FALSE;
   		}
  	}
  	
  	
  	/*******************************************************************************************
  	*
  	*								Config
  	*
  	*******************************************************************************************/
  	
  	/*
  	*	Checks if Config is valid and reads from Config.
  	*/
  	event void Mount.mountDone(error_t error)
  	{
	 
	    if (error == SUCCESS)
	    {
	      	if (call Config.valid() == TRUE)
	      	{	
	      		// Load config
	        	if (call Config.read(CONFIG_ADDR, &conf, sizeof(conf)) != SUCCESS)
	        	{
	          		dbg("Storage","Error: Config read failed\n");
	          		call Leds.set(7);
	   			}
	      	}
	      	else
	      	{
				// Invalid volume.  Commit to make valid.
				dbg("Storage","Error: Storage not valid\n");
				if (call Config.commit() == SUCCESS) 
				{
		  		}
				else
				{
		  			dbg("Storage","Error: Config.commit failed\n");
		  			call Leds.set(7);
		  		}
	      	}
	    }
	    else
	    {
	      // Handle failure
	      call Leds.set(7);
	    }
    }
    
    
    /*
    *	Sets state of mote by Config.
    */
    event void Config.readDone(storage_addr_t addr, void* buf, storage_len_t len, error_t err) __attribute__((noinline)) 
    {
    	
    	if (err == SUCCESS) 
    	{
      		memcpy(&conf, buf, len);
      		
      		if(conf.flashVersion == FLASH_VERSION)
			{	
				// update version 
		      	conf.version = conf.version +1;
		      	collectorHumidityMsg.version 		= conf.version;
				collectorTemperatureMsg.version 	= conf.version;
				collectorLightMsg.version		 	= conf.version;			
			
	      		// note: conf is overwritten from storage
	      		if (conf.state == CONFIG_STATE_INIT) 
	      		{
	      			// start writing to log
	        		conf.state = CONFIG_STATE_WRITING;
	        		call Config.write(CONFIG_ADDR, &conf, sizeof(conf));  
	        		call Leds.set(1);      		
	        		
	      		}
	      		else if (conf.state == CONFIG_STATE_WRITING) 
	      		{
	        		// starts writing to log automaticly by sensor timer
	        		call Leds.set(2);
	        		
	      		}
	      		else if (conf.state == CONFIG_STATE_READING) 
	      		{
	      			// reading from log
	      			call LogRead.seek(call LogRead.currentOffset()); 
	      			call ReadLogTimer.startOneShot(READLOG_INTERVAL);
	      			call Leds.set(3);
	      			
	      		}
      		}
      		else
      		{
	      		// FlashVersion mismatch. Restore default
	      		conf.flashVersion = FLASH_VERSION;
	      		conf.state = CONFIG_STATE_INIT;
	      		conf.version = 0;
	      		call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
	      		// erase flash and then start writing
	      		storageBusy = TRUE;
		      	if (call LogWrite.erase() != SUCCESS) 
		      	{
					// Handle error.
					call Leds.set(7);
		      	}
	      		call Leds.set(4);
	      		
      		}
      		
      		// start sensorTimer
      		startSensorTimer();	
      		
    	}
    	else 
    	{
      		// Handle failure.
    	}
  	}
	
	/*
	*	Calls Config.commit() to ensure data was written.
	*/
  	event void Config.writeDone(storage_addr_t addr, void *buf, storage_len_t len, error_t err) 
  	{
    	// Verify write
    	if (err == SUCCESS) 
    	{
      		if (call Config.commit() != SUCCESS) 
      		{
        		// Handle failure
        		call Leds.set(7);
      		}
    	}
    	else 
    	{
      		// Handle failure
      		call Leds.set(7);
    	}
  	}
	
	/*
	*	Checks if writing to Conf was successful.
	*/
  	event void Config.commitDone(error_t err) 
  	{
    	
    	if (err == SUCCESS) 
    	{
      		// Handle failure
      		call Leds.set(7);
    	}
  	}
  	
  	/*******************************************************************************************
  	*
  	*								Log
  	*
  	*******************************************************************************************/
  	
  	
  	
  	/*
  	*	Enqueues reading to send over radio/serial and erases log if the logEntry isn't valid.
  	*/
    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) 
    {	
    	
    	SensorMsg currentSensorMsg;
        int i;
        
        if ( (len == sizeof(logentry_t)) && (buf == &logEntry) ) 
    	{
    		call Leds.led0Toggle();
      		// build msg to send
      		currentSensorMsg.sensor  	= logEntry.sensor;
		    currentSensorMsg.version 	= logEntry.version;
		    currentSensorMsg.sender   	= TOS_NODE_ID;
    		currentSensorMsg.receiver 	= SERIAL_ADDR;
    		
	     	// fill the sensor readind data
	  		for(i = 0; i < NREADINGS; i++)
	  		{
	  			currentSensorMsg.readings[i] = logEntry.readings[i];
	  		}
      			
      		// send message
		    if(TOS_NODE_ID == 0)
		    {
		    	serialSendSensorMsg(&currentSensorMsg);
			}
			else
			{
				QueueInfo info;
				info.timestamp = GETTIME ;
				info.type = AM_SENSORMSG ;
				radioSendSensorMsg(&currentSensorMsg, info);
			}
			
			// finished reading
			storageBusy = FALSE;
			
			// start next read from log	
			if(conf.state == CONFIG_STATE_READING)
      		{	
      			call ReadLogTimer.startOneShot(READLOG_INTERVAL);
      		}
    	}
    	else 
    	{	
    		// no valid entry -> erase
    		storageBusy = TRUE;
	      	if (call LogWrite.erase() != SUCCESS) 
	      	{
				// Handle error.
	      	}
    	}
  	}
  	
  	/*
  	*	Unfreezes log and starts writing to log:
  	*/
  	event void LogWrite.eraseDone(error_t err) 
  	{	
  		storageBusy = FALSE;
		if (err == SUCCESS) 
		{
			// start sensors and writing to log
        	conf.state = CONFIG_STATE_WRITING;
        	call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
        	
		}
		else 
		{
			// Handle error.
		}
	}
  	
  	/*
  	*	Not in use.
  	*/
  	event void LogRead.seekDone(error_t err) {
  	}
	
	/*
  	*	Not in use.
  	*/
  	event void LogWrite.syncDone(error_t err) {
  	}
    
  	/*
  	*	Makes log writable. 
  	*/
  	event void LogWrite.appendDone(void* buf, storage_len_t len, bool recordsLost, error_t err) 
  	{
    	storageBusy = FALSE;
    	call Leds.led1Toggle();
  	}
  	
   	
  	
  	/*******************************************************************************
  	*
  	*							Timers
  	*
  	*******************************************************************************/
  	
  	/*
  	 * Starts timer with DEFAULT_SAMPLING_INTERVAL to read sensors.
  	 */
  	void startSensorTimer() {
    	call SensorTimer.startPeriodic(DEFAULT_SAMPLING_INTERVAL);
    }
    
    /*
  	 * Starts table timer with TABLE_MSG_INTERVAL to send table message.
  	 */
  	void startTableTimer() {
    	call TableTimer.startPeriodic(TABLE_MSG_INTERVAL);
    }
    
    /*
    *	Initiates one time reading from log
    */ 
    event void ReadLogTimer.fired()
    {	
    	if(!storageBusy)
    	{
	    	storageBusy = TRUE;
	    	
	    	if (call LogRead.read(&logEntry, sizeof(logentry_t)) != SUCCESS) 
	    	{
				// Handle error.
				storageBusy = FALSE;
	      	}   
	    }
	    else
	    {
	   		// to fast reading from log
	   		dbg("Storage", "Reading from log was maybe to fast!");
	   		call Leds.set(7);
	   	}  	
	      	
    }
    
    
    /*
    *	Initiates sending a table message.
    */ 
    event void TableTimer.fired()
    {	
    	enqTable();
    	//printf("+++++ Hallo +++++++");
    	//printfflush();
    }
    
    /* At each sample period:
     - if local sample buffer is full, store accumulated samples
     - read next sample
  	*/
    event void SensorTimer.fired()
    {	
    	int i;
    	// collected all data for this msg?
    	if(readingCountSensorHumidity == NREADINGS)
    	{
    	 	if (!storageBusy)
			{
			    storageBusy  = TRUE;
			    logEntry.sensor = SENSOR_HUMIDITY;
			    logEntry.version = collectorHumidityMsg.version; 
			    
			    for(i = 0; i < NREADINGS; i++)
		  		{
		  			logEntry.readings[i] = collectorHumidityMsg.readings[i];
		  		}
	
			    
			    if (call LogWrite.append(&logEntry, sizeof(logentry_t)) != SUCCESS) 
			    {
			    	storageBusy = FALSE;
			    }
			    
			    // reset count
				readingCountSensorHumidity = 0;
		    }
			
			
    	}
    	if(readingCountSensorTemperature == NREADINGS)
    	{
    		if (!storageBusy)
			{
			    storageBusy  = TRUE;
			    logEntry.sensor = SENSOR_TEMPERATURE;
			    logEntry.version = collectorTemperatureMsg.version; 
			    
			    for(i = 0; i < NREADINGS; i++)
		  		{
		  			logEntry.readings[i] = collectorTemperatureMsg.readings[i];
		  		}
	
			    
			    if (call LogWrite.append(&logEntry, sizeof(logentry_t)) != SUCCESS) 
			    {
			    	storageBusy = FALSE;
			    }
		    		    
				// reset count
				readingCountSensorTemperature = 0;
			}	   		
    	}
    	
    	if(readingCountSensorLight == NREADINGS)
    	{
    	 	if (!storageBusy)
			{
			    storageBusy  = TRUE;
			    logEntry.sensor = SENSOR_LIGHT;
			    logEntry.version = collectorLightMsg.version; 
			    
			    for(i = 0; i < NREADINGS; i++)
		  		{
		  			logEntry.readings[i] = collectorLightMsg.readings[i];
		  		}
	
			    
			    if (call LogWrite.append(&logEntry, sizeof(logentry_t)) != SUCCESS) 
			    {
			    	storageBusy = FALSE;
			    }
		    
			    // reset count
				readingCountSensorLight = 0;
		  
		    }
	   	}
    	
    	// call read of sensor
    	if(conf.state == CONFIG_STATE_WRITING)
    	{
    		call SensorHumidity.read();   	
#ifndef SIMULATION
    		call SensorTemperature.read();
    		call SensorLight.read();
#endif	
    	}
    } 
  	
  	/*
  	*	gets fired when a new beacon broadcast needs to be send
  	*/
  	event void BeaconTimer.fired()
  	{
  		enqBeacon();
  	}
  	
  	
  	/*
  	*	gets fired when the timout for receiving acknowledgements from neighbors must be finished
  	*/
  	event void AckTimer.fired()
  	{
  		int i;
  		bool needRetransmit = FALSE;
  		
  		for(i=0;i<AM_TABLESIZE;i++)
  		{
  			MoteTableEntry *curEntry = &neighborTable[i];
  			
  			if(curEntry->nodeId != AM_MAXNODEID)
  			{
  				dbg("Routing","Table: Node: %d expired: %d ackReceived: %d\n",curEntry->nodeId,curEntry->expired, curEntry->ackReceived);
  				// either the node which got a cmd message isn't present any more or the acknowledgement wasn't received correctly
	  			if(curEntry->expired || !curEntry->ackReceived)
	  			{
	  				needRetransmit = TRUE;
	  				
	  				if(curEntry->expired){
	  					curEntry->nodeId = AM_MAXNODEID;
	  					curEntry->lastContact = 0;
	  					curEntry->expired = FALSE;
	  					curEntry->parentMote = UNDEFINED;
  						curEntry->childMote = FALSE;
  						curEntry->hops = UNDEFINED;
  						curEntry->avgLqi = 0;
	  				}
	  				else
	  				{
	  					//dbg("Routing", "ack timer fired but not expired!\n", curEntry->nodeId);
	  				}
	  				
	  			}
	  		}
	  		else
	  		{
	  			//dbg("Routing", "no entry found!\n");
	  		}
  		}
  		
  		if(needRetransmit)
  		{	
  			//retransmit
  			//dbg("TestSerialC","need retransmit\n");
  			radioSend((CommandMsg*)&rcvRadio);
  		}
  		else
  		{
  			dbg("TestSerialC","successfully got all acks\n");
  		}
  	}
  	
  	
	/*******************************************************************************
  	*
  	*							RadioReceive
  	*
  	*******************************************************************************/
  
  	/*
  	*	This function receives messages received over the radio
	*	forwards all messages which are not targeted to the current node
	*	The message id distinguishes the different message types @see AM_TABLEMSG, AM_COMMANDMSG, AM_SENSORMSG
  	*/
  	event message_t *RadioReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len)
  	{
  		//dbg("TestSerialC","received msg on channel %d\n",id);
  		if(id == AM_BEACONMSG && (sizeof(BeaconMsg)==len))
  		{
  			BeaconMsg *msgReceived;
  			int16_t freeSlot = -1;
  			uint16_t i;
  			bool found = FALSE;
  			uint16_t newAvgLqi; 
  			
  			msgReceived = (BeaconMsg*)payload;
  			
  			//calculate avgLqi
  			if(msgReceived->parent != UNDEFINED)
  			{
  				newAvgLqi = (((msgReceived->avgLqi) * (msgReceived->hops)) + GETRSSI) / (msgReceived->hops + 1) ;
  				
  			}
  			else
  			{
  				newAvgLqi = 0;
  			}
  			
  			// when received a beacon add an entry to the neighbour table and ack
  			for(i=0;i<AM_TABLESIZE;i++)
  			{
  				MoteTableEntry *curEntry = &neighborTable[i];
  				if((curEntry->lastContact == 0))
  				{
  					if(freeSlot == -1)
  					{
  						freeSlot = i;
  						//dbg("TestSerialC","set free slot to: %d\n",freeSlot);
  					}
  				}
  				else
  				{
  					// if there is a entry already there for this node -> update the timestamp
  					if(curEntry->nodeId == msgReceived->sender)
  					{
  						found = TRUE;
  						curEntry->lastContact = GETTIME; // time(NULL); // returns seconds
  						curEntry->parentMote = msgReceived->parent;
  						dbg("Routing","Received Beacon from %d, parent=%d, hops=%d\n",msgReceived->sender, msgReceived->parent,msgReceived->hops);
  						if(msgReceived->parent == TOS_NODE_ID) // mote has selected my mote as parent
  						{
  							curEntry->childMote = TRUE;
  						}
  						else
  						{
  							curEntry->childMote = FALSE;
  						}
  						curEntry->hops = msgReceived->hops;
  						curEntry->avgLqi = msgReceived->avgLqi;
  						
  						//dbg("TestSerialC","curEntry->nodeID: %d found entry for node: %d in neighbor table - update time\n",curEntry->nodeId,msgReceived->sender);
  					}
  					// otherwise delete the node in the table when the timelimit AM_BEACONTIMEOUT is reached
  					else
  					{
  						uint16_t timediff = (GETTIME - curEntry->lastContact) ; // (time(NULL) - curEntry->lastContact);
  						if(timediff > AM_BEACONTIMEOUT)
  						{
  							//dbg("TestSerialC","removed node %d from neighbor table - timediff: %d\n",curEntry->nodeId,timediff);
  							curEntry->expired = TRUE;
  							//if(!nodeBusy)
  							{
  								// connection  to parent is lost ... route failure... reset parent and broadcast lost connection
			  					if(curEntry->nodeId == chosenParent)
			  					{	
			  						dbg("Routing", "######### Connection to mote %d lost!\n", curEntry->nodeId);
			  						localHops = UNDEFINED; 	// lower is better
			 						localAvgLqi = 0;	    // higher is better
			 						chosenParent = UNDEFINED; 	// UNDEFINED = no parent 
			 						localVersion = call Random.rand16();
			 						if(localVersion == UNDEFINED) localVersion--;
			 						enqBeacon();
			  					}
			  					
  								curEntry->nodeId = AM_MAXNODEID;
  								curEntry->lastContact = 0;
  								curEntry->parentMote = UNDEFINED;
		  						curEntry->childMote = FALSE;
		  						curEntry->hops = UNDEFINED;
		  						curEntry->avgLqi = 0;
  							}
  						}
  					}
  				}
  			}
  			if(freeSlot == -1){
  				//dbg("TestSerialC","found NO free entry in neighbor table\n");
  			}
  			// freier slot gefunden
  			else{
  				// aber kein bereits vorhandener knoteneintrag
  				if(!found)
  				{
  					//dbg("TestSerialC","create new entry on position: %d for node %d\n",freeSlot,msgReceived->sender);
  					neighborTable[freeSlot].nodeId = msgReceived->sender;
  					neighborTable[freeSlot].lastContact = GETTIME; //time(NULL);
  					neighborTable[freeSlot].parentMote = msgReceived->parent;
  					if(msgReceived->parent == TOS_NODE_ID) // mote has selected my mote as parent
  					{
  						neighborTable[freeSlot].childMote = TRUE;
  					}
  					else
  					{
  						neighborTable[freeSlot].childMote = FALSE;
  					}
  					neighborTable[freeSlot].hops = msgReceived->hops;
  					neighborTable[freeSlot].avgLqi = msgReceived->avgLqi;
  				}
  			}
  			
  			//****************************************************************************************
  			//
  			//									choose parent
  			//
  			//****************************************************************************************
  			
  			//always update route if msg from parent
  			if(msgReceived->sender == chosenParent)
  			{
  				dbg("Routing","######## msg received from parent: sender=%d, senderParent=%d\n",msgReceived->sender, msgReceived->parent);
  				//new parent
  				if(msgReceived->parent == UNDEFINED)
  				{
  					chosenParent = UNDEFINED;
  					localVersion = msgReceived->version;
  					localAvgLqi = 0;
  					localHops = UNDEFINED;
  				
  					// broadcast new info	
  					enqBeacon();
  				}
  				else
  				{
	  				localVersion = msgReceived->version;
	  				localAvgLqi = newAvgLqi;
	  				localHops 	 = msgReceived->hops + 1;
	  			}  				
  			}
  			
  			//update route if necessary
  			if((msgReceived->parent != TOS_NODE_ID) && (msgReceived->parent != UNDEFINED)) //&&  // msg not from child, avoids loops
  			{
  				if(  (msgReceived->hops + 1 < localHops) // better hopCount
  				|| ((msgReceived->hops+1 == localHops) && (newAvgLqi > localAvgLqi)) )	//same hopCount but better RSSI?
  				{
  					dbg("Routing","update routing information: msgReceived->sender=%d,msgReceived->parent=%d, msgReceived->version=%d, localVersion=%d , hopsMsg=%d, localhops=%d\n",msgReceived->sender,msgReceived->parent,msgReceived->version,localVersion, msgReceived->hops,localHops);
  					//new parent
  					chosenParent = msgReceived->sender;
  					localVersion = msgReceived->version;
  					localAvgLqi = newAvgLqi;
  					localHops 	 = msgReceived->hops + 1;
  				
  					// broadcast new info
  					enqBeacon();
  				}
  			}
  			return msg;
  		}
  		
  		
  		
  		//dbg("TestSerialC","received msg on channel %d\n",id);
  		// got the right message to cast ?
  		if ((id == AM_COMMANDMSG) && (len == sizeof(CommandMsg)))
  		{
    		CommandMsg *msgReceived;
  			memcpy(&rcvRadio,payload,len);
    		msgReceived = (CommandMsg*)&rcvRadio;
    		//dbg("TestSerialC","Node %d received msg for %d from %d isAck: %d\n",TOS_NODE_ID,msgReceived->receiver,msgReceived->sender,msgReceived->isAck);
    		
			if(msgReceived->receiver == TOS_NODE_ID)
			{
				if(msgReceived->isAck)
				{
					int i;
					//dbg("TestSerialC","Node %d received Ack from: %d\n",TOS_NODE_ID,msgReceived->sender);
					for(i=0;i<AM_TABLESIZE;i++)
					{
						// update the neighbor table and set ackReceived to TRUE
						MoteTableEntry *curEntry = &neighborTable[i];
						if(curEntry->nodeId == msgReceived->sender)
						{
							curEntry->ackReceived = TRUE;
							break;
						}
					}
					return msg;
				}
    			else if(msgReceived->seqNum > localSeqNumberCommand)
    			{
    				dbg("TestSerialC","Finished Node %d: received message on RadioChannel seqNum: %d\n",TOS_NODE_ID,msgReceived->seqNum);
    				localSeqNumberCommand = msgReceived->seqNum;
    			    //	call Leds.set(msgReceived->ledNum);
    				
    				// start reading/sending of log
    				if(msgReceived->reqSensor == 1)
					{
						conf.state = CONFIG_STATE_READING;
						call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
						call Leds.led1Toggle();
						call ReadLogTimer.startOneShot(READLOG_INTERVAL);
    				}
    				// stop reading/sending of log
    				if(msgReceived->reqSensor == 0)
    				{
    					conf.state = CONFIG_STATE_WRITING;
    					call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
    				}
    				
    				// start reading/sending of log
    				if(msgReceived->reqSensor == 2)
					{
						conf.state = CONFIG_STATE_INIT;
						call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
						call LogWrite.erase();
    				}
    				
    				// enable/disable led
    				call Leds.set(msgReceived->ledNum);
    			}
    			else
    			{
	    			dbg("TestSerialC","Node %d:duplicate message received from %d\n",TOS_NODE_ID,msgReceived->sender);
	    		}
	    		//post sendRadioAck();
				enqAck();
				post sendQueueTask();
	    	}
	    	else
	    	{	
	    		if(msgReceived->seqNum > localSeqNumberCommand)
	    		{
	    			localSeqNumberCommand = msgReceived->seqNum;
    				radioSend(msgReceived);
    			}
    			//post sendRadioAck();
	    		enqAck();
	    		post sendQueueTask();
    		}
		}
		if((id == AM_TABLEMSG) && (len == sizeof(TableMsg)))
		{
			TableMsg* curMsg;
			am_addr_t source;
			
			// if its node 0 then send over serial to pc, if not forward the message
			curMsg = (TableMsg*)payload;
			source = call RadioAMPacket.source(msg);
			
			//dbg("TestSerialC","received tablemsg over radio from source: %d, sender: %d\n",source,curMsg->sender);
			
				
			if(TOS_NODE_ID == 0)
			{
				//dbg("TestSerialCSerial","forward table message from %d to serial parent: %d\n",curMsg->sender,curMsg->parent);
				//curMsg->seqNum = curMsg->seqNum + 1;
				serialSendTable((TableMsg*)curMsg);
				post serialSendQueueTask();
			}
			else if(curMsg->sender != TOS_NODE_ID)
			{
				QueueInfo info;
				//call Leds.led0Toggle();
	 			forwardTable((TableMsg*)curMsg, info);
	 			post sendQueueTask();
				//dbg("TestSerialC","Seq no conflict!!! seqReceived: %d, seqInTable: %d\n",curMsg->seqNum,seqNums[curMsg->sender]);
			}
			
		}
		// if we have received a SensorMsg then forward the message
		if((id == AM_SENSORMSG) && (len == sizeof(SensorMsg)))
		{	
			SensorMsg* curMsg;
			am_addr_t source;
			
			// if its node 0 then send over serial to pc, if not forward the message
			curMsg = (SensorMsg*)payload;
			source = call RadioAMPacket.source(msg);
			
			
			if(TOS_NODE_ID == 0)
			{
				dbg("TestSerialCSensor","node 0 received sensor data - forward to serial\n");
				serialSendSensorMsg((SensorMsg*)curMsg);
				post serialSendQueueTask();
			}
			else
			{	
				QueueInfo info;
				info.type = AM_SENSORMSG;
				info.timestamp = GETTIME;
				dbg("TestSerialC","forward Sensor message from %d to %d\n",curMsg->sender,curMsg->receiver);
				radioSendSensorMsg((SensorMsg*)curMsg, info);
				post sendQueueTask();
			}
		}
    	return msg;
  	}// end RadioReceive.receiver
  	
  	
  	/*
  	*	Event which gets fired after successful/failed transmission of a message over radio
  	*/
  	event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error)
  	{
    	if (error != SUCCESS)
    	{
    		dbg("TestSerialC","Error: Node %d couldnt send message on RadioChannel\n",TOS_NODE_ID);
    	}
    	else
    	{
    		//dbg("TestSerialC","send done for ch id: %d\n",id);
    		if(id == AM_COMMANDMSG)
    		{
    			CommandMsg* sentMsg;
    			//dbg("TestSerialC","sent CommandMsg\n");
	    		if(TOS_NODE_ID == 0)
	    		{
	    			//call Leds.led0Toggle();
	    		}
 			
	 			// start a timer within all neighbors in the table must acknowledge the receival
	 			sentMsg = (CommandMsg*)&sndRadioLast;
	 			//dbg("TestSerialC","SendDone: sentMsg->isAck: %d, receiver: %d, sender: %d\n",sentMsg->isAck,sentMsg->receiver,sentMsg->sender);
	 			if(!sentMsg->isAck)
	 			{
	 				//dbg("TestSerialC","send done for normal message-> start timer\n");
	 				call AckTimer.startOneShot( AM_ACKTIMEOUT );
	 			}
	 		}else if(id == AM_BEACONMSG)
	 		{
	 			//dbg("TestSerialC","sent BeaconMsg\n");
	 			
	 		}
	 		else if(id == AM_TABLEMSG)
			{
	 			TableMsg* sentMsg;
	 			bool acked = call PacketAcknowledgements.wasAcked(msg);
	 			
	 			if(!acked && checkForAck)
	 			{
	 				QueueInfo info;
	 				info.type = AM_TABLEMSG;
	 				info.timestamp = GETTIME;
	 				sentMsg = (TableMsg*)&sndRadioLast;
	 				dbg("Acked","retransmit tableMsg %d\n",acked);
	 				forwardTable((TableMsg*)&sndRadioLast,info);
	 			}
	 			//dbg("TestSerialC","sent TableMsg\n");
	 			//dbg("TestSerialC","SendDone for tableMsg: receiver: %d, sender: %d\n",sentMsg->receiver,sentMsg->sender);
			}
			else if (id == AM_SENSORMSG)
			{
				bool acked;
				acked = call PacketAcknowledgements.wasAcked(msg);
	 			if(!acked && checkForAck)
	 			{	
	 				QueueInfo info;
	 				info.type = AM_SENSORMSG;
	 				info.timestamp = GETTIME;
	 				dbg("Acked","retransmit sensorMsg %d\n",acked);
	 				radioSendSensorMsg((SensorMsg*)&sndRadioLast, info);
	 			}
			}
		}
		radioBusy = FALSE;
		if(call RadioMsgPool.put(msg) == SUCCESS)
		{
			//dbg("Pool","elements in RadioPool: %d\n",(call RadioMsgPool.size()));
			//dbg("Pool","elements in SerialPool: %d\n",(call SerialMsgPool.size()));
		}
		else
		{
			dbg("Pool","could't free element in radioPool\n");
		}
		post sendQueueTask();
  	} // end RadioSend.sendDone
  	
  	
  	
  	
  	
  	
  	
  	  	
  	/*
  	*	Sends a CommandMsg over Radio
  	*
  	*	@param CommandMsg to send
  	*/
  	void radioSend(CommandMsg *receivedMsgToSend)
  	{
		// is radio unused?
  		message_t *newMsg = call RadioMsgPool.get();
  		CommandMsg* msgToSend;
  		if(newMsg == NULL)
  		{
  			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
  	    	return;
  		}
		
		msgToSend = (CommandMsg*)(call RadioPacket.getPayload(newMsg, sizeof (CommandMsg)));
		msgToSend->sender 	= TOS_NODE_ID;
		msgToSend->seqNum 	= receivedMsgToSend->seqNum;
		msgToSend->ledNum 	= receivedMsgToSend->ledNum;
		msgToSend->receiver = receivedMsgToSend->receiver;
		msgToSend->reqSensor = receivedMsgToSend->reqSensor;
		msgToSend->isAck 	= 0;
	
		//dbg("TestSerialC","got cmd forward msg from pool\n");
		
		if (call RadioQueue.enqueue(newMsg) != SUCCESS)
		{
			dbg("TestSerialC","couldnt enqueue Command msg\n");
		}
		else
		{
			QueueInfo info;
			info.type = AM_COMMANDMSG;
			//dbg("TestSerialC","enqueued new beacon msg\n");
			call RadioTypeQueue.enqueue(info);
			post sendQueueTask();
		}
  	}
  	
  	/*
  	*	sends a certain TableMsg over the radio channel via broadcast
  	*
  	*	@param TableMsg to send over radio
  	*/
  	void forwardTable(TableMsg* msg, QueueInfo info)
  	{
		message_t* newMsg;
		TableMsg* tblMsg;
		
  		if(chosenParent == UNDEFINED)
		{
			dbg("Routing","no parent - do not send tableMsg\n");
			return;
		}
		
		newMsg = call RadioMsgPool.get();
  		if(newMsg == NULL)
		{
			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
			return;
		}

  		tblMsg = call RadioPacket.getPayload(newMsg, sizeof (TableMsg));
		memcpy(tblMsg,msg,sizeof(TableMsg));
		//dbg("TestSerialC","enqueue forwarded table message from pool\n");

		if (call RadioQueue.enqueue(newMsg) != SUCCESS)
		{
			dbg("TestSerialC","couldnt enqueue forwarded table msg\n");
		}
		else{
			info.type = AM_TABLEMSG;
			call RadioTypeQueue.enqueue(info);
			post sendQueueTask();
		}
	
  	}
  	
  	/*
  	*	Sends a SensorMsg over radio.
  	*	@param receivedMsgToSend pointer to received message
  	*/
  	void radioSendSensorMsg(SensorMsg* msg, QueueInfo info)
  	{	
  		SensorMsg* sensorMsg;
  		message_t *newMsg;
  		if(chosenParent == UNDEFINED)
		{
  			dbg("Routing","wont forward sensor msg -> no parent node\n");
  			return;
  		}
		
		newMsg = call RadioMsgPool.get();
  		if(newMsg == NULL)
  		{
  			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
  			return;
  		}
  		sensorMsg = call RadioPacket.getPayload(newMsg, sizeof (SensorMsg));
		memcpy(sensorMsg,msg,sizeof(SensorMsg));
		
		dbg("TestSerialCSensor","radio send sensor start\n");
	
		//dbg("TestSerialCSensor","got sensor msg from pool\n");
		// forward or send message
		if (call RadioQueue.enqueue(newMsg) != SUCCESS)
		{
			dbg("TestSerialC","couldnt enqueue forwarded table msg\n");
			call RadioMsgPool.put(newMsg);
		}
		else{
			//QueueInfo info;
			info.type = AM_SENSORMSG;
			call RadioTypeQueue.enqueue(info);
			post sendQueueTask();
		}
  	}
  	
  	
  	
  	
  	
  	/*******************************************************************************
  	*
  	*							Serial
  	*
  	*******************************************************************************/
  	
  	/*
  	*	Forwards a certain message on the Serial Port -  applies only to Node 0
  	*/
  	void serialSendTable(TableMsg* msg)
  	{
		int i;
		message_t *newMsg = call SerialMsgPool.get();
		TableMsg* msgToSend;
  		if(newMsg == NULL)
  		{
  			dbg("TestSerialC","Error: No free serial msg pointer in pool!\n");
  			return;
  		}
		msgToSend = (TableMsg*)(call SerialPacket.getPayload(newMsg, sizeof (TableMsg)));
		msgToSend->sender = msg->sender;
		msgToSend->receiver = msg->receiver;
		msgToSend->parent = msg->parent;
		msgToSend->avgLqi = msg->avgLqi;
		
		for(i=0;i<AM_TABLESIZE;i++)
		{
			msgToSend->nodeId[i] = msg->nodeId[i];
			msgToSend->lastContact[i] = msg->lastContact[i];
			//dbg("TestSerialC","before sending over serial nodeId: %d lastContact: %d\n",msgToSend->nodeId[i],msgToSend->lastContact[i]);
		}
				
		// forward message
		if(call SerialQueue.enqueue(newMsg) == SUCCESS)
		{
			call SerialTypeQueue.enqueue(AM_TABLEMSG);
			//dbg("TestSerialC","serial reflect\n");
		}
		else
		{
			call SerialMsgPool.put(newMsg);
		}
  	}
  	
  	/*
  	*	Sends a SensorMsg over serial.
  	* 	@param inputMsg SensorMsg pointer to message which should be sent.
  	*/
  	void serialSendSensorMsg(SensorMsg *msg)
  	{
  		//dbg("TestSerialCSensor","send sensor data over serial\n");
  		message_t *newMsg = call SerialMsgPool.get();
  		SensorMsg* msgToSend;
  		if(newMsg == NULL)
  		{
  			dbg("TestSerialC","Error: No free serial msg pointer in pool!\n");
  			return;
  		}
  		
		msgToSend = (SensorMsg*)(call SerialPacket.getPayload(newMsg, sizeof (SensorMsg)));
		memcpy(msgToSend,msg,sizeof(SensorMsg));
	
		// forward or send message
		if(call SerialQueue.enqueue(newMsg) == SUCCESS)
		{
			call SerialTypeQueue.enqueue(AM_SENSORMSG);
			post serialSendQueueTask();
		}
		else
		{
			call SerialMsgPool.put(newMsg);
		}
  	}
  	
	/*
	*	Sends the received message from pc directly back to the pc (used as a message received indication)
	*/
  	void serialReflect() 
  	{	
		CommandMsg* msgReceived = (CommandMsg*)&rcvSerial;
		message_t *newMsg = call SerialMsgPool.get();
		CommandMsg* msgToSend;
  		if(newMsg == NULL)
  		{
  			dbg("TestSerialC","Error: No free serial msg pointer in pool!\n");
  			return;
  		}
		
		msgToSend = (CommandMsg*)(call SerialPacket.getPayload(newMsg, sizeof (CommandMsg)));
		msgToSend->sender 	= TOS_NODE_ID;
		msgToSend->seqNum 	= msgReceived->seqNum;
		msgToSend->ledNum 	= msgReceived->ledNum;
		msgToSend->receiver = msgReceived->sender;
		msgToSend->isAck 	= TRUE;
	
		// forward message
		if(call SerialQueue.enqueue(newMsg) == SUCCESS)
		{
			call SerialTypeQueue.enqueue(AM_COMMANDMSG);
			//dbg("TestSerialC","serial reflect\n");
		}
  	}
  	
  	/*
  	*	Event which is fired after a transmit of a message over the serial channel
  	*/
  	event void SerialSend.sendDone[am_id_t id](message_t* msg, error_t error)
  	{
	    if (error == SUCCESS)
	    {
	  		// nice
	    }
	    serialBusy = FALSE;
  		//dbg("TestSerialC", "error on message pointer\n");
  		call SerialMsgPool.put(msg);
  		post serialSendQueueTask();
  	}
  	
  	
  	
  	/*
  	*	Event which is fired when a certain message was received over the serial channel
  	*	reflects the received message to the gui application indication successful reception
  	*	and forwads the received command over radio
  	*/
  	event message_t *SerialReceive.receive[am_id_t id](message_t *msg, void *payload, uint8_t len)
  	{  	
  		dbg("TestSerialC","received message on serial channel\n");	
  		// got the right message to cast ?
  		if (len == sizeof(CommandMsg))
  		{
    		CommandMsg *msgReceived;
  			memcpy(&rcvSerial,payload,len);
    		msgReceived = (CommandMsg*)&rcvSerial;
    		    		
    		// check sequence number to avoid sending of duplicates
    		if(msgReceived->seqNum > localSeqNumberCommand)
    		{
    			localSeqNumberCommand=msgReceived->seqNum;
    			
    			if(msgReceived->receiver == TOS_NODE_ID)
    			{
    				
    				
	    			//dbg("TestSerialC","Finished Node %d: received message on RadioChannel seqNum: %d\n",TOS_NODE_ID,msgReceived->seqNum);
	    			localSeqNumberCommand = msgReceived->seqNum;
	    			// start reading/sending of log
    				if(msgReceived->reqSensor == 1)
					{
						conf.state = CONFIG_STATE_READING;
						call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
						call ReadLogTimer.startOneShot(READLOG_INTERVAL);
    				}
    				// stop reading/sending of log
    				if(msgReceived->reqSensor == 0)
    				{
    					conf.state = CONFIG_STATE_WRITING;
    					call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
    				}
    				// start reading/sending of log
    				if(msgReceived->reqSensor == 2)
					{
						conf.state = CONFIG_STATE_INIT;
						call Config.write(CONFIG_ADDR, &conf, sizeof(conf));
						call LogWrite.erase();
    				}

    				// enable/disable led
    				call Leds.set(msgReceived->ledNum);
    				
    				
	    			//send a CommandMsg over the serial Channel
    				serialReflect();
    				post serialSendQueueTask();
    			}
    			else
    			{
    				message_t *newMsg = call RadioMsgPool.get();
    				if(newMsg != NULL)
    				{
						// inject the packet into the sensornet		
						CommandMsg* msgToSend = (CommandMsg*)(call RadioPacket.getPayload(newMsg, sizeof (CommandMsg)));
						
						msgToSend->sender 		= TOS_NODE_ID;
						msgToSend->seqNum 		= msgReceived->seqNum;
						msgToSend->ledNum 		= msgReceived->ledNum;
						msgToSend->receiver 	= msgReceived->receiver;
						msgToSend->reqSensor   	= msgReceived->reqSensor;
						msgToSend->isAck 		= 0; 
						
						// enqueue message
						if(call RadioQueue.enqueue(newMsg) == SUCCESS)
						{
							QueueInfo info;
							info.type = AM_COMMANDMSG;
							call RadioTypeQueue.enqueue(info);
							post sendQueueTask();
						}
						serialReflect();
						post serialSendQueueTask();
    				}
    				else{
    					dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
    				}
				}
    		}
  		}	  		
  		return msg;
  	}
  	
  	
  	
  	/*******************************************************************************
  	*
  	*						Queue: send serial/radio , enqueue
  	*
  	*******************************************************************************/
  	/*
  	*	Enqueues a message and sends it over radio.
  	*/
  	task void sendQueueTask()
  	{
  		if(radioBusy){
  			//dbg("TestSerialC","Queue: busy radio->return\n");
  			return;
  		}
  		if(call RadioQueue.empty() == FALSE)
  		{
  			message_t *qMsg = call RadioQueue.dequeue();
  			QueueInfo info = call RadioTypeQueue.dequeue();
  			uint8_t type = info.type;
  			int msgLen = 0;

  			am_addr_t receiver = AM_BROADCAST_ADDR;
  			
  			//dbg("TestSerialC","Queue: enqueued element with type %d\n",type);
  			  			
  			switch(type)
  			{
				case AM_BEACONMSG:
				{
					BeaconMsg *sent = (BeaconMsg*)(call RadioPacket.getPayload(qMsg, sizeof (BeaconMsg))); // jump to starting pointer
					msgLen = sizeof(BeaconMsg);
					receiver = AM_BROADCAST_ADDR;
					//dbg("TestSerialC","Queue: Start sending BeaconMsg\n");
					memcpy(&sndRadioLast,sent,msgLen);
					break;
				}
				case AM_SENSORMSG:
				{
					SensorMsg *sent = (SensorMsg*)(call RadioPacket.getPayload(qMsg, sizeof (SensorMsg))); // jump to starting pointer
					msgLen = sizeof(SensorMsg);
					
					receiver = chosenParent;
					
					if((call PacketAcknowledgements.requestAck(qMsg)) == SUCCESS)
					{
						checkForAck = TRUE;
					}
					else
					{
						checkForAck = FALSE;
					}
					//dbg("TestSerialC","Queue: Start sending SensorMsg\n");
					memcpy(&sndRadioLast,sent,msgLen);
					break;
				}
				case AM_TABLEMSG:
				{
					TableMsg *sent = (TableMsg*)(call RadioPacket.getPayload(qMsg, sizeof (TableMsg))); // jump to starting pointer
					msgLen = sizeof(TableMsg);
					
					receiver = chosenParent;

					if((call PacketAcknowledgements.requestAck(qMsg)) == SUCCESS)
					{
						checkForAck = TRUE;
					}
					else
					{
						checkForAck = FALSE;
					}
					//dbg("TestSerialC","Queue: Start sending TableMsg from %d to %d\n",sent->sender,sent->receiver);
					//dbg("TestSerialC","Queue: receiver: %d sndRadio-sender: %d\n",sent->receiver,sent->sender);
					memcpy(&sndRadioLast,sent,msgLen);
					break;
				}
				case AM_COMMANDMSG:
				{
					CommandMsg *sent = (CommandMsg*)(call RadioPacket.getPayload(qMsg, sizeof (CommandMsg))); // jump to starting pointer
					msgLen = sizeof(CommandMsg);
					//dbg("TestSerialC","Queue: Start sending CommandMsg\n");
					//dbg("TestSerialC","Queue: receiver: %d sndRadio-sender: %d sndRadio-ack: %d\n",sent->receiver,sent->sender, sent->isAck);
					memcpy(&sndRadioLast,sent,msgLen);
					if(sent->isAck)
						receiver = sent->receiver;
					else
						receiver = AM_BROADCAST_ADDR;
					break;
				}
  			}
  			if (call RadioSend.send[type](receiver, qMsg, msgLen) != SUCCESS) {
  			 	dbg("TestSerialC","Error Sending");
  			}
  			else{
  				radioBusy = TRUE;
  			}
  		}
  		else
  		{
  			//dbg("TestSerialC","Queue: no element in the queue\n");
  		}
  	}
  	
  	/*
  	*	Enqueues a message and sends it over serial.
  	*/
  	task void serialSendQueueTask()
	{
		if(serialBusy){
			//dbg("TestSerialC","Queue: busy radio->return\n");
			return;
		}
		if(call SerialQueue.empty() == FALSE)
		{
			message_t *qMsg = call SerialQueue.dequeue();
			uint8_t type = call SerialTypeQueue.dequeue();
		
			am_addr_t receiver = 99;
			uint8_t msgLen = 0;
			
			//dbg("TestSerialC","Queue: enqueued element with type %d\n",type);
			switch(type)
			{
				case AM_BEACONMSG:
				{
					BeaconMsg *sent = (BeaconMsg*)(call SerialPacket.getPayload(qMsg, sizeof (BeaconMsg))); // jump to starting pointer
					msgLen = sizeof(BeaconMsg);
					memcpy(&sndSerialLast,sent,msgLen);
					break;
				}
				case AM_SENSORMSG:
				{
					SensorMsg *sent = (SensorMsg*)(call SerialPacket.getPayload(qMsg, sizeof (SensorMsg))); // jump to starting pointer
					msgLen = sizeof(SensorMsg);
					memcpy(&sndSerialLast,sent,msgLen);
					break;
				}
				case AM_TABLEMSG:
				{
					TableMsg *sent = (TableMsg*)(call SerialPacket.getPayload(qMsg, sizeof (TableMsg))); // jump to starting pointer
					msgLen = sizeof(TableMsg);
					memcpy(&sndRadioLast,sent,msgLen);
					break;
				}
				case AM_COMMANDMSG:
				{
					CommandMsg *sent = (CommandMsg*)(call SerialPacket.getPayload(qMsg, sizeof (CommandMsg))); // jump to starting pointer
					msgLen = sizeof(CommandMsg);
					memcpy(&sndSerialLast,sent,msgLen);
				}
			}
			if (call SerialSend.send[type](receiver, qMsg, msgLen) != SUCCESS) {
				dbg("TestSerialC","Error Sending");
			}
			else{
				serialBusy = TRUE;
			}
		}
		else
		{
			//dbg("TestSerialC","Queue: no element in the queue\n");
		}
	}
  	/*
  	*	Sends an acknowledgement to the sender of the last received command
  	*/
  	void enqAck()
  	{
		CommandMsg* lastMsg = (CommandMsg*)&rcvRadio;
		CommandMsg* msgToSend;
		message_t *newMsg = call RadioMsgPool.get();
		if(newMsg == NULL)
		{
			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
			return;
		}
		msgToSend = (CommandMsg*)(call RadioPacket.getPayload(newMsg, sizeof (CommandMsg)));
	
		msgToSend->sender = TOS_NODE_ID;
		msgToSend->seqNum = lastMsg->seqNum;
		msgToSend->ledNum = lastMsg->ledNum;
		msgToSend->receiver = lastMsg->sender;
		msgToSend->isAck = 1;	

		//dbg("TestSerialC","send ack to: %d from %d\n",lastMsg->sender,TOS_NODE_ID);

		if (call RadioQueue.enqueue(newMsg) != SUCCESS)
		{
			dbg("TestSerialC","couldnt enqueue msg\n");
		}
		else
		{
			QueueInfo info;
			info.type = AM_COMMANDMSG;
			call RadioTypeQueue.enqueue(info);
		}
  	}
  	

 	/*
  	*	Sends a BeaconMsg over Radio via broadcast
  	*/
  	void enqBeacon()
  	{
  		message_t *newMsg = call RadioMsgPool.get();
  		BeaconMsg* msgToSend;
  		if(newMsg == NULL)
		{
			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
			return;
		}
  		
		msgToSend = (BeaconMsg*)(call RadioPacket.getPayload(newMsg, sizeof (BeaconMsg)));
		msgToSend->sender 	= TOS_NODE_ID;
		msgToSend->hops 	= localHops;
		msgToSend->avgLqi 	= localAvgLqi;
		msgToSend->version 	= localVersion;
		msgToSend->parent 	= chosenParent;
		
		//dbg("TestSerialC","got beacon msg from pool\n");
		
		if (call RadioQueue.enqueue(newMsg) != SUCCESS)
		{
			dbg("TestSerialC","couldnt enqueue beacon msg\n");
		}
		else
		{
			QueueInfo info;
			info.type = AM_BEACONMSG;
			//dbg("TestSerialC","enqueued new beacon msg\n");
			call RadioTypeQueue.enqueue(info);
			post sendQueueTask();
		}
  	}
  	
  	/*
  	*	Task to forward table message to other nodes
  	*/
  	void enqTable()
  	{
		int i;
		message_t *newMsg;
		TableMsg* msgToSend;
  		if(chosenParent == UNDEFINED)
		{
			dbg("TestSerialC","Error: No free radio msg pointer in pool!\n");
			return;
		}
		newMsg = call RadioMsgPool.get();
		if( newMsg == NULL )
		{
			return;
		}
		msgToSend = (TableMsg*)(call RadioPacket.getPayload(newMsg, sizeof (TableMsg)));
		
		if(msgToSend == NULL)
		{
			dbg("TestSerialC","null pointer on msg struct\n");
			call RadioMsgPool.put(newMsg);
			return;
		}
		msgToSend->sender 	= TOS_NODE_ID;
		msgToSend->receiver = SERIAL_ADDR;
		msgToSend->avgLqi 	= localAvgLqi;
		msgToSend->parent 	= chosenParent;
		
		//dbg("TestSerialC","start sending tableMsg\n");
		
		for(i=0;i<AM_TABLESIZE;i++)
		{
			//dbg("TestSerialC","fill no %d: nodeId: %d lastCont: %d\n",i,neighborTable[i].nodeId,neighborTable[i].lastContact);
			msgToSend->nodeId[i] = neighborTable[i].nodeId;
			msgToSend->lastContact[i] = neighborTable[i].lastContact;
		}
		//dbg("TestSerialC","after filling tableMsg\n");
		
		if(TOS_NODE_ID == 0)
		{
			serialSendTable(msgToSend);
			// important to free memory if its node 0, where no sending over radio is performed
			if(call RadioMsgPool.put(newMsg) != SUCCESS)
			{
				dbg("TestSerialC","couldn't free table memory after serial send\n");
			}
			post serialSendQueueTask();
		}
		else
		{
			if (call RadioQueue.enqueue(newMsg) != SUCCESS)
			{
				dbg("TestSerialC","couldnt enqueue table msg\n");
			}
			else
			{
				QueueInfo info;
				info.type = AM_TABLEMSG;
				//dbg("TestSerialC","enqueued new table msg\n");
				call RadioTypeQueue.enqueue(info);
				post sendQueueTask();
			}
		}
  	}
  	
  	
  	
  	/*******************************************************************************
  	*
  	*				Sensors: init and read 
  	*
  	********************************************************************************/

 
     /*
     * Inits all sensors messages and readingCounts.
     */
     void initSensors()
     {
     	int i;
     	dbg("TestSerialCSensor","initSensors\n");
     	
     	
    	collectorHumidityMsg.sender 	= TOS_NODE_ID;
    	collectorHumidityMsg.receiver 	= SERIAL_ADDR;
    	collectorHumidityMsg.sensor 	= SENSOR_HUMIDITY;
    	collectorHumidityMsg.version 	= 0;
                 	
     	
    	collectorTemperatureMsg.sender 	= TOS_NODE_ID;
    	collectorTemperatureMsg.receiver 	= SERIAL_ADDR;
    	collectorTemperatureMsg.sensor 	= SENSOR_TEMPERATURE;	
    	collectorTemperatureMsg.version 	= 0;
         	
     	
    	collectorLightMsg.sender 		= TOS_NODE_ID;
    	collectorLightMsg.receiver 	= SERIAL_ADDR;
    	collectorLightMsg.sensor 		= SENSOR_LIGHT;	
      	collectorLightMsg.version 		= 0;
     	
     	
     	// initialize the sensor readind data
  		for(i = 0; i < NREADINGS; i++)
  		{
  			collectorHumidityMsg.readings[i] = UNDEFINED;
  		}
     	
     	for(i = 0; i < NREADINGS; i++)
  		{
  			collectorTemperatureMsg.readings[i] = UNDEFINED;
  		}
     	
     	for(i = 0; i < NREADINGS; i++)
  		{
  			collectorLightMsg.readings[i] = UNDEFINED;
  		}
  		
  		
  		readingCountSensorHumidity = 0;
		readingCountSensorTemperature = 0;
		readingCountSensorLight = 0;
  			
     }
  	
  	/*
  	*	Gets data from sensor.
  	* 	Writes data into the message.
  	*	If reading was not successful then data = UNDEFINED.
  	*/
  	event void SensorHumidity.readDone(error_t result, uint16_t data) {
  		
  		// successful?
	    if (result != SUCCESS)
	    {
			data = UNDEFINED;
	    }
	    
	    // add data to message
	    if (readingCountSensorHumidity < NREADINGS)
	    { 
	      	collectorHumidityMsg.readings[readingCountSensorHumidity++] = data;
	  	}
  	}
  	
  	
  	/*
  	*	Gets data from sensor.
  	* 	Writes data into the message.
  	*	If reading was not successful then data = UNDEFINED.
  	*/
  	event void SensorTemperature.readDone(error_t result, uint16_t data) {
  		
  		// successful?
	    if (result != SUCCESS)
	    {
			data = UNDEFINED;
	    }
	    
	    // add data to message
	    if (readingCountSensorTemperature < NREADINGS)
	    { 
	      	collectorTemperatureMsg.readings[readingCountSensorTemperature++] = data;
	  	}
  	}
  	
  	/*
  	*	Gets data from sensor.
  	* 	Writes data into the message.
  	*	If reading was not successful then data = UNDEFINED.
  	*/
  	event void SensorLight.readDone(error_t result, uint16_t data) {
  		
  		// successful?
	    if (result != SUCCESS)
	    {
			data = UNDEFINED;
	    }
	    
	    // add data to message
	    if (readingCountSensorLight < NREADINGS)
	    { 
	      	collectorLightMsg.readings[readingCountSensorLight++] = data;
	  	}
  	}
  	
  	
  	
  	
  	
  	/*******************************************************************************
  	*
  	*								Not in use
  	*
  	********************************************************************************/
  	 	
  	
  	/*
  	*	Not in use.
  	*/
  	async event void ActiveMessageAddress.changed() 
  	{
  	}
  	
  	/*
  	*	Not in use.
  	*/
  	event void RadioControl.startDone(error_t error) 
  	{
    	if (error == SUCCESS) {
    		//dbg("TestSerialC","start done\n");
    	}
  	}
	
	/*
  	*	Not in use.
  	*/
  	event void SerialControl.startDone(error_t error) 
  	{
    	if (error == SUCCESS) {
    		//dbg("TestSerialC","start done\n");
    	}
  	}
	
	/*
  	*	Not in use.
  	*/
  	event void SerialControl.stopDone(error_t error)
  	{
  	}
  	
  	/*
  	*	Not in use.
  	*/
  	event void RadioControl.stopDone(error_t error)
  	{
  	}
  	
} 
