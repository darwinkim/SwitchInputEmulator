import serial
import time
import math

class Session():
    # Compute x and y based on angle and intensity
    def angle(self, angle, intensity):
        # y is negative because on the Y input, UP = 0 and DOWN = 255
        x = int((math.cos(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
        y = -int((math.sin(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
        return x, y

    def lstick_angle(self, angle, intensity):
        return (intensity + (angle << 8)) << 24

    def rstick_angle(self, angle, intensity):
        return (intensity + (angle << 8)) << 44

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

    # Convert DPAD value to actual DPAD value used by Switch
    def decrypt_dpad(self, dpad):
        if dpad == self.DIR_U:
            dpadDecrypt = self.A_DPAD_U
        elif dpad == self.DIR_R:
            dpadDecrypt = self.A_DPAD_R
        elif dpad == self.DIR_D:
            dpadDecrypt = self.A_DPAD_D
        elif dpad == self.DIR_L:
            dpadDecrypt = self.A_DPAD_L
        elif dpad == self.DIR_U_R:
            dpadDecrypt = self.A_DPAD_U_R
        elif dpad == self.DIR_U_L:
            dpadDecrypt = self.A_DPAD_U_L
        elif dpad == self.DIR_D_R:
            dpadDecrypt = self.A_DPAD_D_R
        elif dpad == self.DIR_D_L:
            dpadDecrypt = self.A_DPAD_D_L
        else:
            dpadDecrypt = self.A_DPAD_CENTER
        return dpadDecrypt

    # Convert CMD to a packet
    def cmd_to_packet(self, command):
        cmdCopy = command
        low = (cmdCopy & 0xFF);
        cmdCopy = cmdCopy >> 8
        high = (cmdCopy & 0xFF);
        cmdCopy = cmdCopy >> 8
        dpad = (cmdCopy & 0xFF);
        cmdCopy = cmdCopy >> 8
        lstick_intensity = (cmdCopy & 0xFF);
        cmdCopy = cmdCopy >> 8
        lstick_angle = (cmdCopy & 0xFFF);
        cmdCopy = cmdCopy >> 12
        rstick_intensity = (cmdCopy & 0xFF);
        cmdCopy = cmdCopy >> 8
        rstick_angle = (cmdCopy & 0xFFF)
        dpad = self.decrypt_dpad(dpad)
        left_x, left_y = self.angle(lstick_angle, lstick_intensity)
        right_x, right_y = self.angle(rstick_angle, rstick_intensity)

        packet = [high, low, dpad, left_x, left_y, right_x, right_y, 0x00]
        # print (hex(command), packet, lstick_angle, lstick_intensity, rstick_angle, rstick_intensity)
        return packet

    # Send a formatted controller command to the MCU
    def send_cmd(self, command=0):
        commandSuccess = self.send_packet(self.cmd_to_packet(command))
        return commandSuccess

    # Test all buttons except for home and capture
    def testbench_btn(self):
        self.send_cmd(self.BTN_A);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_B);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_X);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_Y);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_PLUS);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_MINUS);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_LCLICK);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.BTN_RCLICK);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)

    # Test DPAD U / R / D / L
    def testbench_dpad(self):
        self.send_cmd(self.DPAD_U);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_R);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_D);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_L);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)

    # Test DPAD Diagonals - Does not register on switch due to dpad buttons
    def testbench_dpad_diag(self):
        self.send_cmd(self.DPAD_U_R);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_D_R);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_D_L);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.DPAD_U_L);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)

    # Test Left Analog Stick
    def testbench_lstick(self):
        # Test U/R/D/L
        self.send_cmd(self.BTN_LCLICK);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.LSTICK_U);
        self.p_wait(0.5)
        self.send_cmd(self.LSTICK_R);
        self.p_wait(0.5)
        self.send_cmd(self.LSTICK_D);
        self.p_wait(0.5)
        self.send_cmd(self.LSTICK_L);
        self.p_wait(0.5)
        self.send_cmd(self.LSTICK_U);
        self.p_wait(0.5)
        self.send_cmd(self.LSTICK_CENTER);
        self.p_wait(0.5)

        # 360 Circle @ Full Intensity
        for i in range(0, 721):
            cmd = self.lstick_angle(i + 90, 0xFF)
            self.send_cmd(cmd)
            self.p_wait(0.001)
        self.send_cmd(self.LSTICK_CENTER);
        self.p_wait(0.5)

        # 360 Circle @ Partial Intensity
        for i in range(0, 721):
            cmd = self.lstick_angle(i + 90, 0x80)
            self.send_cmd(cmd)
            self.p_wait(0.001)
        self.send_cmd(self.LSTICK_CENTER);
        self.p_wait(0.5)

    # Test Right Analog Stick
    def testbench_rstick(self):
        # Test U/R/D/L
        self.send_cmd(self.BTN_RCLICK);
        self.p_wait(0.5);
        self.send_cmd();
        self.p_wait(0.001)
        self.send_cmd(self.RSTICK_U);
        self.p_wait(0.5)
        self.send_cmd(self.RSTICK_R);
        self.p_wait(0.5)
        self.send_cmd(self.RSTICK_D);
        self.p_wait(0.5)
        self.send_cmd(self.RSTICK_L);
        self.p_wait(0.5)
        self.send_cmd(self.RSTICK_U);
        self.p_wait(0.5)
        self.send_cmd(self.RSTICK_CENTER);
        self.p_wait(0.5)

        # 360 Circle @ Full Intensity
        for i in range(0, 721):
            cmd = self.rstick_angle(i + 90, 0xFF)
            self.send_cmd(cmd)
            self.p_wait(0.001)
        self.send_cmd(self.RSTICK_CENTER);
        self.p_wait(0.5)

        # 360 Circle @ Partial Intensity
        for i in range(0, 721):
            cmd = self.rstick_angle(i + 90, 0x80)
            self.send_cmd(cmd)
            self.p_wait(0.001)
        self.send_cmd(self.RSTICK_CENTER);
        self.p_wait(0.5)

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

    def testbench(self):
        self.testbench_btn()
        self.testbench_dpad()
        self.testbench_lstick()
        self.testbench_rstick()
        self.testbench_packet_speed()
        return

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

    def __init__(self):
        self.STATE_OUT_OF_SYNC = 0
        self.STATE_SYNC_START = 1
        self.STATE_SYNC_1 = 2
        self.STATE_SYNC_2 = 3
        self.STATE_SYNC_OK = 4

        # Actual Switch DPAD Values
        self.A_DPAD_CENTER = 0x08
        self.A_DPAD_U = 0x00
        self.A_DPAD_U_R = 0x01
        self.A_DPAD_R = 0x02
        self.A_DPAD_D_R = 0x03
        self.A_DPAD_D = 0x04
        self.A_DPAD_D_L = 0x05
        self.A_DPAD_L = 0x06
        self.A_DPAD_U_L = 0x07

        # Enum DIR Values
        self.DIR_CENTER = 0x00
        self.DIR_U = 0x01
        self.DIR_R = 0x02
        self.DIR_D = 0x04
        self.DIR_L = 0x08
        self.DIR_U_R = self.DIR_U + self.DIR_R
        self.DIR_D_R = self.DIR_D + self.DIR_R
        self.DIR_U_L = self.DIR_U + self.DIR_L
        self.DIR_D_L = self.DIR_D + self.DIR_L

        self.BTN_NONE = 0x0000000000000000
        self.BTN_Y = 0x0000000000000001
        self.BTN_B = 0x0000000000000002
        self.BTN_A = 0x0000000000000004
        self.BTN_X = 0x0000000000000008
        self.BTN_L = 0x0000000000000010
        self.BTN_R = 0x0000000000000020
        self.BTN_ZL = 0x0000000000000040
        self.BTN_ZR = 0x0000000000000080
        self.BTN_MINUS = 0x0000000000000100
        self.BTN_PLUS = 0x0000000000000200
        self.BTN_LCLICK = 0x0000000000000400
        self.BTN_RCLICK = 0x0000000000000800
        self.BTN_HOME = 0x0000000000001000
        self.BTN_CAPTURE = 0x0000000000002000

        self.DPAD_CENTER = 0x0000000000000000
        self.DPAD_U = 0x0000000000010000
        self.DPAD_R = 0x0000000000020000
        self.DPAD_D = 0x0000000000040000
        self.DPAD_L = 0x0000000000080000
        self.DPAD_U_R = self.DPAD_U + self.DPAD_R
        self.DPAD_D_R = self.DPAD_D + self.DPAD_R
        self.DPAD_U_L = self.DPAD_U + self.DPAD_L
        self.DPAD_D_L = self.DPAD_D + self.DPAD_L

        self.LSTICK_CENTER = 0x0000000000000000
        self.LSTICK_R = 0x00000000FF000000  # 0 (000)
        self.LSTICK_U_R = 0x0000002DFF000000  # 45 (02D)
        self.LSTICK_U = 0x0000005AFF000000  # 90 (05A)
        self.LSTICK_U_L = 0x00000087FF000000  # 135 (087)
        self.LSTICK_L = 0x000000B4FF000000  # 180 (0B4)
        self.LSTICK_D_L = 0x000000E1FF000000  # 225 (0E1)
        self.LSTICK_D = 0x0000010EFF000000  # 270 (10E)
        self.LSTICK_D_R = 0x0000013BFF000000  # 315 (13B)

        self.RSTICK_CENTER = 0x0000000000000000
        self.RSTICK_R = 0x000FF00000000000  # 0 (000)
        self.RSTICK_U_R = 0x02DFF00000000000  # 45 (02D)
        self.RSTICK_U = 0x05AFF00000000000  # 90 (05A)
        self.RSTICK_U_L = 0x087FF00000000000  # 135 (087)
        self.RSTICK_L = 0x0B4FF00000000000  # 180 (0B4)
        self.RSTICK_D_L = 0x0E1FF00000000000  # 225 (0E1)
        self.RSTICK_D = 0x10EFF00000000000  # 270 (10E)
        self.RSTICK_D_R = 0x13BFF00000000000  # 315 (13B)

        self.NO_INPUT = self.BTN_NONE + self.DPAD_CENTER + self.LSTICK_CENTER + self.RSTICK_CENTER

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


        # self.

    def initport(self):
        ser = serial.Serial(port="COM4", baudrate=19200, timeout=1)