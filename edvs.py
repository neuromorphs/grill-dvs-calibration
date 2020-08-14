import numpy as np
import threading
import atexit
import time

class Serial(object):
    def __init__(self, port, baud):
        import serial
        self.conn = serial.Serial(port, baudrate=baud, rtscts=True, timeout=0)
    def send(self, message):
        self.conn.write(message.encode('utf-8'))
    def receive(self):
        return self.conn.read(1024)
    def close(self):
        self.conn.close()

import socket
class Socket(object):
    cache = {}
    def __init__(self, address, port=56000):
        self.socket = Socket.get_socket(address, port)

    @classmethod
    def get_socket(cls, address, port):
        key = (address, port)
        s = cls.cache.get(key, None)
        if s is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((address, port))
            s.settimeout(0)
            cls.cache[key] = s
        return s

    def send(self, message):
        self.socket.send(message.encode())
    def receive(self):
        try:
            return self.socket.recv(1024)
        except socket.error:
            return b''
    def close(self):
        self.socket.close()


class EDVS:
    def __init__(self):
        self.connection = None
        self.retina_packet_size = None
        self.image = None
        self.record_file = None

    def connect(self, connection):
        self.connection = connection
        self.last_time = {}
        self.connection.send('\n')
        self.retina(False)
        atexit.register(self.disconnect)
        thread = threading.Thread(target=self.sensor_loop)
        thread.daemon = True
        thread.start()
        
    def disconnect(self):
        self.retina(False)
        if self.record_file is not None:
            self.record_file.close()
        self.connection.close()

    def retina(self, active, bytes_in_timestamp=4):
        if active:
            assert bytes_in_timestamp in [0, 2, 3, 4]
            cmd = '!E%d\nE+\n' % bytes_in_timestamp
            self.retina_packet_size = 2 + bytes_in_timestamp
        else:
            cmd = 'E-\n'
            self.retina_packet_size = None
        self.connection.send(cmd)

    def show_image(self, decay=0.5, display_mode='quick'):
        if self.image is None:
            self.image = np.zeros((128, 128), dtype=float)
            thread = threading.Thread(target=self.image_loop,
                                      args=(decay, display_mode))
            thread.daemon = True
            thread.start()

    def image_loop(self, decay, display_mode):
        import pylab

        import matplotlib.pyplot as plt
        # using axis for updating only parts of the image that change
        fig, ax = plt.subplots()
        # so quick mode can run on ubuntu
        plt.show(block=False)

        pylab.ion()
        img = pylab.imshow(self.image, vmax=1, vmin=-1,
                                       interpolation='none', cmap='binary')
        pylab.xlim(0, 127)
        pylab.ylim(127, 0)

        while True:

            img.set_data(self.image)

            if display_mode == 'quick':
                # this is faster, but doesn't work on all systems
                fig.canvas.draw()
                fig.canvas.flush_events()
                
            elif display_mode == 'ubuntu_quick':
                # this is even faster, but doesn't work on all systems
                ax.draw_artist(ax.patch)
                ax.draw_artist(img)
                ax.draw_artist(scatter)
                fig.canvas.update()

                fig.canvas.flush_events()
            else:
                # this works on all systems, but is kinda slow
                pylab.pause(1e-8)

            self.image *= decay


    def sensor_loop(self):
        """Handle all data coming from the robot."""
        old_data = None
        buffered_ascii = b''
        while True:
            packet_size = self.retina_packet_size
            # grab the new data
            data = self.connection.receive()
            if len(data) > 0:
                print(len(data))

            # combine it with any leftover data from last time through the loop
            if old_data is not None:
                data = old_data + data
                old_data = None

            if packet_size is None:
                # no retina events, so everything should be ascii
                buffered_ascii += data
            else:
                # find the ascii events
                data_all = np.frombuffer(data, np.uint8)
                ascii_index = np.where(data_all[::packet_size] < 0x80)[0]

                offset = 0
                while len(ascii_index) > 0:
                    # if there's an ascii event, remove it from the data
                    index = ascii_index[0]*packet_size
                    stop_index = np.where(data_all[index:] >=0x80)[0]
                    if len(stop_index) > 0:
                        stop_index = index + stop_index[0]
                    else:
                        stop_index = len(data)

                    # and add it to the buffered_ascii list
                    buffered_ascii += data[offset+index:offset+stop_index]
                    data_all = np.hstack((data_all[:index],
                                          data_all[stop_index:]))
                    offset += stop_index - index
                    ascii_index = np.where(data_all[::packet_size] < 0x80)[0]

                # handle any partial retina packets
                extra = len(data_all) % packet_size
                if extra != 0:
                    old_data = data[-extra:]
                    data_all = data_all[:-extra]
                if len(data_all) > 0:
                    # now process those retina events
                    self.process_retina(data_all)

            # and process the ascii events too
            while b'\n' in buffered_ascii:
                cmd, buffered_ascii = buffered_ascii.split(b'\n', 1)
                self.process_ascii(cmd)

    def process_ascii(self, message):
        message = message.decode('utf-8')
        print(message)

    last_timestamp = None
    def process_retina(self, data):
        packet_size = self.retina_packet_size
        y = data[::packet_size] & 0x7f
        x = data[1::packet_size] & 0x7f
        if self.record_file is not None:
            self.record_file.write(data)
        if self.image is not None:
            value = np.where(data[1::packet_size]>=0x80, 1, -1)
            np.add.at(self.image, (y, x), value)


    def record_retina_data(self, filename):
        self.record_file = open(filename, 'wb')
        
        
if __name__ == '__main__':
    edvs = EDVS()
    edvs.connect(Socket('99.250.220.231', port=9105))
    #edvs.connect(Serial('COM6', baud=4000000))
    time.sleep(1)
    edvs.retina(True)
    edvs.show_image(display_mode='quick', decay=0.2)
    while True:
        time.sleep(0.01)