#socket_echo_client_dgram.py

import socket
import sys
import time

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('127.0.0.1', 8080)

start_time= time.time()

while 1==1:
#print("time=",time.time()-start_time)
    	if time.time()-start_time<100:
		x=4-(time.time()-start_time)*0.3
		y=4-(time.time()-start_time)*0.3 #vehicle.velocity=1??
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
    		print('closing socket')
    		sock.close()


