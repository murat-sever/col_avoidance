#socket_echo_client_dgram.py

import socket
import sys
import time

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('127.0.0.1', 8080)

start_time= time.time()
#time.sleep(6)
i=0

while 1==1:
	#if i == 0:
	#	xr=1.735
	#	yr=-0.5387
	#	zr=-0.110987
	#if i == 1:
	#	xr=1.6725
	#	yr=-0.4745
	#	zr=-0.066
	#if i == 2:
	#	xr=1.533
	#	yr=-0.414
	#	zr=-0.06942
	#if i == 3:
	#	xr=1.7283
	#	yr=-0.306589
	#	zr=-0.118257
	#if i == 4:
	#	xr=1.802136
	#	yr=-0.2398
	#	zr=-0.1008
	#if i == 5:
	#	xr=1.83
	#	yr=-0.2045
	#	zr=-0.1107
	#if i == 6:
	#	xr=1.894
	#	yr=-0.1464
	#	zr=-0.086
	#if i == 7:
	#	xr=1.837
	#	yr=-0.1248
	#	zr=-0.168
	#if i == 8:
	#	xr=1.8
	#	yr=-0.2045
	#	zr=-0.1107
	#if i == 9:
	#	xr=1.8
	#	yr=-0.1464
	#	zr=-0.086
	if i == 0:
		xr=0
		zr=0.5248
		yr=-0.168
	if i == 1:
		xr=0
		zr=0.3
		yr=-0.168
	if i == 2:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 3:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 4:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 5:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 6:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 7:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 8:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 9:
		xr=0
		zr=0.01
		yr=-0.168
	if i == 10:
		xr=0
		zr=0.01
		yr=-0.1107
	if i == 11:
		xr=0
		zr=0.01
		yr=0.086
	if i == 12:
		xr=0
		zr=0.01464
		yr=0.168
	if i == 13:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 14:
		xr=0
		zr=0.48
		yr=0.168
	if i == 15:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 16:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 17:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 18:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 19:
		xr=0
		zr=-0.3248
		yr=0.168
	if i == 20:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 21:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 22:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 23:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 24:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 25:
		xr=0
		zr=0.01248
		yr=0.168
	if i == 26:
		xr=0
		zr=0.01248
		yr=-0.68
	if i == 27:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 28:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 29:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 30:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 31:
		xr=0
		zr=0.01248
		yr=-0.168
	if i == 32:
		xr=0
		zr=-0.55
		yr=-0.168
	if i == 33:
		xr=0
		zr=-0.01248
		yr=-0.168
	if i == 34:
		xr=0
		zr=-0.01248
		yr=-0.168
	if i == 35:
		xr=0
		zr=-0.248
		yr=-0.168
	if i == 36:
		xr=0
		zr=0
		yr=0
	if i == 37:
		xr=0
		zr=0
		yr=0
	if i == 38:
		xr=0
		zr=0
		yr=0
	message_string = "%f %f %f" % (xr, yr, zr)
	message = message_string.encode('UTF-8') 

	try:
		sent = sock.sendto(message, server_address)
		print("time=",time.time()-start_time)
		#seconds = time.time()
		#print("Seconds=", seconds)
	

	finally:
	     time.sleep(1)
	     i=i+1
#	     print('closing socket')
#	     sock.close()
	
	


