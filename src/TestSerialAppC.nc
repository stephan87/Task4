#include "TestSerial.h"
#include "StorageVolumes.h"

configuration TestSerialAppC {}
implementation {
  components TestSerialC as App, LedsC, MainC;
  components SerialActiveMessageC as AMSerial;
  components ActiveMessageC as AMRadio;
  components new TimerMilliC() as BeaconTimer;
  components new TimerMilliC() as AckTimer;
  components new TimerMilliC() as SensorTimer; 
  components new TimerMilliC() as ReadLogTimer;
  components new QueueC(message_t*, 10) as RadioQueueC;
  components new PoolC(message_t, 10) as RadioMsgPoolC;
  components new QueueC(QueueInfo, 10) as RadioTypeQueueC;
  components new QueueC(message_t*, 5) as SerialQueueC;
  components new QueueC(uint8_t, 5) as SerialTypeQueueC;
  components new PoolC(message_t, 5) as SerialMsgPoolC;
  components RandomC;
  components ActiveMessageAddressC;
  components new ConfigStorageC(VOLUME_CONFIGVOLUME);
  components new LogStorageC(VOLUME_SENSORLOGVOLUME, TRUE);
  
#ifndef SIMULATION
  components LocalTimeSecondC;
  components new SensirionSht11C() as SensorHumidityTemperature;
  components new HamamatsuS1087ParC() as SensorLight;
  components CC2420ActiveMessageC;
#else
  components new DemoSensorC() as TossimDemoSensor;
#endif

  App.Boot -> MainC.Boot;
  
  App.SerialControl  -> AMSerial;
  App.SerialSend 	 -> AMSerial;
  App.SerialReceive  -> AMSerial.Receive;
  App.SerialPacket 	 -> AMSerial;
  App.SerialAMPacket -> AMSerial;
  
  App.RadioControl 	-> AMRadio;
  App.RadioSend 	-> AMRadio;
  App.RadioReceive 	-> AMRadio.Receive;
  App.RadioPacket 	-> AMRadio;
  App.RadioAMPacket -> AMRadio;
  
  App.BeaconTimer 	-> BeaconTimer;
  App.AckTimer 		-> AckTimer;
  App.SensorTimer 	-> SensorTimer;
  App.Leds 			-> LedsC;
  App.Random 		-> RandomC;
  App.RadioQueue 	-> RadioQueueC;
  App.RadioTypeQueue -> RadioTypeQueueC;
  App.RadioMsgPool 	-> RadioMsgPoolC;
  App.SerialQueue 	-> SerialQueueC;
  App.SerialTypeQueue -> SerialTypeQueueC;
  App.SerialMsgPool -> SerialMsgPoolC;
  App.PacketAcknowledgements -> AMRadio.PacketAcknowledgements;
  App.ActiveMessageAddress -> ActiveMessageAddressC;
  
  App.Config     -> ConfigStorageC.ConfigStorage;
  App.Mount      -> ConfigStorageC.Mount;
  
  App.LogRead -> LogStorageC;
  App.LogWrite -> LogStorageC;
  App.ReadLogTimer -> ReadLogTimer;
  
#ifndef SIMULATION
  App.LocalTime -> LocalTimeSecondC;
  App.SensorHumidity -> SensorHumidityTemperature.Humidity;
  App.SensorTemperature -> SensorHumidityTemperature.Temperature;
  App.SensorLight -> SensorLight;
  App.CC2420Packet -> CC2420ActiveMessageC;
#else
  App.SensorHumidity -> TossimDemoSensor;
#endif
}
