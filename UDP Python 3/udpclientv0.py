#socket_echo_client_dgram.py

import socket
import sys
import time

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('127.0.0.1', 8080)

start_time= time.time()
i=0

while 1==1:
#print("time=",time.time()-start_time)
	#if time.time()-start_time<10:
	#     x=1-(time.time()-start_time)*0.1
	#     y=1-(time.time()-start_time)*0.1 #vehicle.velocity=1??
	#else:
	#     x=(time.time()-start_time)*0.05
	#     y=(time.time()-start_time)*0.05
	seconds = time.time()
	print("Seconds since epoch =", seconds)
	if i < 4:
		if i % 4 == 0:
			x=-1
			y=-1
		if i % 4 == 1:
			x=1
			y=-1
		if i % 4 == 2:
			x=1
			y=1
		if i % 4 == 3:
			x=-1
			y=1
	elif 3 < i < 8:
		if i % 4 == 0:
			x=0
			y=1
		if i % 4 == 1:
			x=1
			y=0
		if i % 4 == 2:
			x=0
			y=-1
		if i % 4 == 3:
			x=-1
			y=0
	else:
		x=0
		y=0
	message_string = "%f %f" % (x, y)
	message = message_string.encode('UTF-8')

	try:
		# Send data
	     print('sending {!r}'.format(message))
	     sent = sock.sendto(message, server_address)
	

	finally:
	     time.sleep(3)
	     i=i+1
#	     print('closing socket')
#	     sock.close()
	
	


