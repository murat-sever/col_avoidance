#socket_echo_client_dgram.py

import socket
import sys

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('127.0.0.1', 8080)

x = -0.898
y = 0.252
message_string = "%f %f" % (x, y)

message = message_string.encode('UTF-8')

try:

    # Send data
    print('sending {!r}'.format(message))
    sent = sock.sendto(message, server_address)


finally:
    print('closing socket')
    sock.close()


