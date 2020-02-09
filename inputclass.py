import serial
import time
import math


class Session():
    # Precision wait
    def p_wait(self, waitTime):
        t0 = time.perf_counter()
        t1 = t0
        while (t1 - t0 < waitTime):
            t1 = time.perf_counter()

    # Wait for data to be available on the serial port
    def wait_for_data(self, timeout=1.0, sleepTime=0.1):
        t0 = time.perf_counter()
        t1 = t0
        inWaiting = self.ser.in_waiting
        while ((t1 - t0 < sleepTime) or (inWaiting == 0)):
            time.sleep(sleepTime)
            inWaiting = self.ser.in_waiting
            t1 = time.perf_counter()

    # Read X bytes from the serial port (returns list)
    def read_bytes(self, size):
        bytes_in = self.ser.read(size)
        return list(bytes_in)

    # Read 1 byte from the serial port (returns int)
    def read_byte(self):
        bytes_in = self.read_bytes(1)
        if len(bytes_in) != 0:
            byte_in = bytes_in[0]
        else:
            byte_in = 0
        return byte_in

    # Discard all incoming bytes and read the last (latest) (returns int)
    def read_byte_latest(self):
        inWaiting = self.ser.in_waiting
        if inWaiting == 0:
            inWaiting = 1
        bytes_in = self.read_bytes(inWaiting)
        if len(bytes_in) != 0:
            byte_in = bytes_in[0]
        else:
            byte_in = 0
        return byte_in

    # Write bytes to the serial port
    def write_bytes(self, bytes_out):
        self.ser.write(bytearray(bytes_out))
        return

    # Write byte to the serial port
    def write_byte(self, byte_out):
        self.write_bytes([byte_out])
        return

    # Compute CRC8
    # https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html
    def crc8_ccitt(self, old_crc, new_data):
        data = old_crc ^ new_data

        for i in range(8):
            if (data & 0x80) != 0:
                data = data << 1
                data = data ^ 0x07
            else:
                data = data << 1
            data = data & 0xff
        return data

    # Send a raw packet and wait for a response (CRC will be added automatically)
    def send_packet(self, packet=[0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00], debug=False):
        if not debug:
            bytes_out = []
            bytes_out.extend(packet)

            # Compute CRC
            crc = 0
            for d in packet:
                crc = self.crc8_ccitt(crc, d)
            bytes_out.append(crc)
            self.write_bytes(bytes_out)
            # print(bytes_out)

            # Wait for USB ACK or UPDATE NACK
            byte_in = self.read_byte()
            commandSuccess = (byte_in == self.RESP_USB_ACK)
        else:
            commandSuccess = True
        return commandSuccess

    def cmd_to_packet(self, high=0, low=0, dpad=8, lx=0x80, ly=0x80, rx=0x80, ry=0x80):
        packet = [high, low, dpad, lx, ly, rx, ry, 0x00]
        # print (hex(command), packet, lstick_angle, lstick_intensity, rstick_angle, rstick_intensity)
        return packet

    # Test Packet Speed
    def testbench_packet_speed(self, count=100, debug=False):
        sum = 0
        min = 999
        max = 0
        avg = 0
        err = 0

        for i in range(0, count + 1):

            # Send packet and check time
            t0 = time.perf_counter()
            status = self.send_packet()
            t1 = time.perf_counter()

            # Count errors
            if not status:
                err += 1
                print('Packet Error!')

            # Compute times
            delta = t1 - t0
            if delta < min:
                min = delta
            if delta > max:
                max = delta
            sum = sum + (t1 - t0)

        avg = sum / i
        print('Min =', '{:.3f}'.format(min), 'Max =', '{:.3}'.format(max), 'Avg =', '{:.3f}'.format(avg), 'Errors =',
              err)

    # Force MCU to sync
    def force_sync(self):
        # Send 9x 0xFF's to fully flush out buffer on device
        # Device will send back 0xFF (RESP_SYNC_START) when it is ready to sync
        self.write_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])

        # Wait for serial data and read the last byte sent
        self.wait_for_data()
        byte_in = self.read_byte_latest()

        # Begin sync...
        inSync = False
        if byte_in == self.RESP_SYNC_START:
            self.write_byte(self.COMMAND_SYNC_1)
            byte_in = self.read_byte()
            if byte_in == self.RESP_SYNC_1:
                self.write_byte(self.COMMAND_SYNC_2)
                byte_in = self.read_byte()
                if byte_in == self.RESP_SYNC_OK:
                    inSync = True
        return inSync

    # Start MCU syncing process
    def sync(self):
        inSync = False

        # Try sending a packet
        inSync = self.send_packet()
        if not inSync:
            # Not in sync: force resync and send a packet
            inSync = self.force_sync()
            if inSync:
                inSync = self.send_packet()
        return inSync

    def __init__(self, port):
        self.ser = serial.Serial(port, baudrate=19200, timeout=1)

        self.STATE_OUT_OF_SYNC = 0
        self.STATE_SYNC_START = 1
        self.STATE_SYNC_1 = 2
        self.STATE_SYNC_2 = 3
        self.STATE_SYNC_OK = 4

        # Commands to send to MCU
        self.COMMAND_NOP = 0x00
        self.COMMAND_SYNC_1 = 0x33
        self.COMMAND_SYNC_2 = 0xCC
        self.COMMAND_SYNC_START = 0xFF

        # Responses from MCU
        self.RESP_USB_ACK = 0x90
        self.RESP_UPDATE_ACK = 0x91
        self.RESP_UPDATE_NACK = 0x92
        self.RESP_SYNC_START = 0xFF
        self.RESP_SYNC_1 = 0xCC
        self.RESP_SYNC_OK = 0x33

        self.state = State()

        if not self.sync():
            print('Could not sync!')

        if not self.send():
            print('Packet Error!')

    def close(self):
        self.ser.close()

    def send(self):
        return self.send_packet([
            1 * self.state.M + 2 * self.state.P + 4 * self.state.LB + 8 * self.state.RB + 16 * self.state.H + 32 * self.state.C,
            1 * self.state.Y + 2 * self.state.B + 4 * self.state.A + 8 * self.state.X + 16 * self.state.L + 32 * self.state.R + 64 * self.state.ZL + 128 * self.state.ZR,
            self.state.dpad, self.state.lx, self.state.ly, self.state.rx, self.state.ry, 0
        ])

    def run(self, **kwargs):
        self.state = State(**kwargs)
        return self.send()

    def press(self, **kwargs):
        if 't' in kwargs:
            t = kwargs['t']
            del kwargs['t']
        else:
            t = 0.2
        old_state = self.state
        self.state = State(**kwargs)
        self.send()
        self.p_wait(t)
        self.state = old_state
        return self.send()


class State:
    def __init__(self, M=False, P=False, LB=False, RB=False, H=False, C=False,
                 Y=False, B=False, A=False, X=False, L=False, R=False, ZL=False, ZR=False,
                 dpad=8, lx=128, ly=128, rx=128, ry=128):
        self.M = M
        self.P = P
        self.LB = LB
        self.RB = RB
        self.H = H
        self.C = C

        self.Y = Y
        self.B = B
        self.A = A
        self.X = X
        self.L = L
        self.R = R
        self.ZL = ZL
        self.ZR = ZR

        self.dpad = dpad

        self.lx = lx
        self.ly = ly
        self.rx = rx
        self.ry = ry
