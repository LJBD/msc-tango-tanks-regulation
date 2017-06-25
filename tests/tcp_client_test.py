from __future__ import print_function
import socket
import struct
from time import sleep

if __name__ == '__main__':
    sock = socket.socket()
    server_address = '192.168.0.23'
    port = 4567
    sock.connect((server_address, port))
    sock.send(struct.pack(">d", 123.456))
    sleep(0.1)
    data = sock.recv(1024)
    print("Received %d signs: %s" % (len(data), data))
    sock.close()
