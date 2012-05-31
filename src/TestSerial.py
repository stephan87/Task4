#!/usr/bin/python
import sys
import time

from TOSSIM import *
#from TestSerialMsg import *

nodes = 4
t = Tossim([])
m = t.mac()
r = t.radio()
sf = SerialForwarder(9002)
#throttle = Throttle(t, 10)

t.addChannel("TestSerialC", sys.stdout);
#t.addChannel("TestSerialCSensor", sys.stdout);
#t.addChannel("Routing",sys.stdout);
t.addChannel("Pool",sys.stdout);
t.addChannel("Acked",sys.stdout);
t.addChannel("TestSerialCSerial",sys.stdout);

f = open("topo.txt", "r")
for line in f:
	s = line.split()
	if s:
		r.add(int(s[0]), int(s[1]), float(s[2]))

noise = open("meyer-heavy-short.txt", "r")
for line in noise:
  s = line.strip()
  if s:
    val = int(s)
    for i in range(0,nodes):
      t.getNode(i).addNoiseTraceReading(val)

for i in range(0,nodes):
  t.getNode(i).createNoiseModel()

for i in range(0, nodes):
  m = t.getNode(i);
  m.turnOn();
  print "node "+str(i)+" is on: "+str(m.isOn())

for i in range (0, 1000000):
	t.runNextEvent();
	time.sleep(0.01);
	#if i == 1200:
	#	t.getNode(1).turnOff();
	sf.process();










	
"""
sf.process();
throttle.initialize();

for i in range(0, 60):
  throttle.checkThrottle();
  t.runNextEvent();
  sf.process();

msg = TestSerialMsg()
msg.set_counter(7);

serialpkt = t.newSerialPacket();
serialpkt.setData(msg.data)
serialpkt.setType(msg.get_amType())
serialpkt.setDestination(0)
serialpkt.deliver(0, t.time() + 3)

pkt = t.newPacket();
pkt.setData(msg.data)
pkt.setType(msg.get_amType())
pkt.setDestination(0)
pkt.deliver(0, t.time() + 10)

for i in range(0, 20):
  throttle.checkThrottle();
  t.runNextEvent();
  sf.process();

throttle.printStatistics()
"""
