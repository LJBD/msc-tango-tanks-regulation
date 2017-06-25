from __future__ import print_function
import logging
import socket
import signal
import struct
from datetime import datetime
from functools import partial
from multiprocessing import Process, Pipe

from ds_tanks.tanks_utils import signal_handler, setup_logging, \
    setup_logging_to_file


class TcpTanksServer(Process):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, pipe_connection, address="0.0.0.0", port=4567,
                 logger=None, log_file_name=None, log_level=logging.DEBUG,
                 name=None):
        super(TcpTanksServer, self).__init__(name=name)
        self.address = address
        self.port = port
        self.pipe_connection = pipe_connection
        formatter = logging.Formatter('%(asctime) - %(name)s - %(level)s:'
                                      '%(message)s')
        if isinstance(logger, logging.Logger):
            self.logger = logger
            if log_file_name:
                handler = setup_logging_to_file(formatter, log_file_name)
                self.logger.addHandler(handler)
        else:
            self.logger = setup_logging(log_file_name, formatter)
        self.logger.setLevel(log_level)

    def run(self):
        self.server_socket.bind((self.address, self.port))
        self.server_socket.listen(1)
        while True:
            self.server_loop()

    def server_loop(self):
        # Wait for a connection
        self.logger.info('Waiting for a connection...')
        connection, client_address = self.server_socket.accept()
        signal.signal(signal.SIGINT, partial(signal_handler,
                                             connection=connection))
        self.logger.info('Connection from', client_address)
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
                    real_data = struct.unpack(">dddd", data)
                except struct.error:
                    real_data = (data,)
                self.logger.debug('received "%s", timestamp: %s' %
                                  (real_data[0], datetime.now()))
                if self.pipe_connection.poll():
                    self.send_data_to_client(connection)
            except socket.error as e:
                self.logger.error("Socket error:", e)
                self.logger.info('No more data from %s:%d, overall packets'
                                 'transmitted: %d ' % (client_address[0],
                                                       client_address[1],
                                                       packet_number))
                break

    def close_connection(self, connection):
        connection.close()
        self.logger.info("Connection closed")

    def send_data_to_client(self, connection):
        self.logger.info('Sending data to the client')
        try:
            data_from_ds = self.pipe_connection.recv()
        except EOFError:
            self.logger.error("ERROR: no data in pipe!")
        else:
            data_format = '>' + 'd' * len(data_from_ds)
            connection.sendall(struct.pack(data_format, *data_from_ds))

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    first_connection, second_connection = Pipe()
    tcp_server = TcpTanksServer(second_connection, name="TcpTanksServer",
                                log_file_name="tcp_tanks_server.log")
    sample_data = [30.0, 30.0, 22.0, 140.312246999846, 100.0, 0,
                   128.23238467535595, 132.87848556939056]
    tcp_server.start()
    first_connection.send(sample_data)
