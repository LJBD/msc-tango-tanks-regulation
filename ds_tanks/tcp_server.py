from __future__ import print_function
import socket
import struct
from datetime import datetime
from multiprocessing import Process
        

class TcpTanksServer(Process):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, pipe_connection, address="0.0.0.0", port=4567,
                 group=None, target=None, name=None):
        super(TcpTanksServer, self).__init__(group=group, target=target,
                                             name=name)
        self.address = address
        self.port = port
        self.pipe_connection = pipe_connection
        self.server_socket.setblocking(False)

    def run(self):
        self.server_socket.bind((self.address, self.port))
        self.server_socket.listen(1)
        while True:
            self.server_loop()

    def server_loop(self):
        # Wait for a connection
        print('Waiting for a connection...')
        connection, client_address = self.server_socket.accept()
        try:
            print('Connection from', client_address)
            if self.pipe_connection.poll():
                self.send_data_to_client(connection)
            else:
                connection.sendall(struct.pack('>d', 0))
            packet_number = 0

            while True:
                try:
                    data = connection.recv(1024)
                    packet_number += 1
                    try:
                        real_data = struct.unpack(">d", data)
                    except struct.error:
                        real_data = (data,)
                    print('received "%s", timestamp: %s' % (real_data[0],
                                                            datetime.now()))
                    if self.pipe_connection.poll():
                        self.send_data_to_client(connection)
                except socket.error as e:
                    print("Socket error:", e)
                    print('No more data from %s:%d, overall packets'
                          'transmitted: %d ' % (client_address[0],
                                                client_address[1],
                                                packet_number))
                    break
        except KeyboardInterrupt:
            print("Got Ctrl+C, closing...")
            connection.close()
            print("Connection closed")
        finally:
            # Clean up the connection
            connection.close()

    def send_data_to_client(self, connection):
        print('Sending data to the client')
        try:
            data_from_ds = self.pipe_connection.recv()
        except EOFError:
            print("ERROR: no data in pipe!")
        else:
            connection.sendall(struct.pack('>d', data_from_ds))
