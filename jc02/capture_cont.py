import serial, time

port = serial.Serial('COM23', 9600, timeout=5)
time.sleep(0.2)

cmd_cont = bytes([0xAE, 0xA7, 0x04, 0x00, 0x0E, 0x12, 0xBC, 0xBE])
port.write(cmd_cont)
print('Continuous measurement started. Collecting for 15 seconds...')
print('>>> Point the sensor at something CLOSE (< 1m) now <<<')
print()

start = time.time()
all_bytes = bytearray()
while time.time() - start < 15:
    n = port.in_waiting
    if n > 0:
        all_bytes += port.read(n)
    time.sleep(0.05)

cmd_stop = bytes([0xAE, 0xA7, 0x04, 0x00, 0x0F, 0x13, 0xBC, 0xBE])
port.write(cmd_stop)
port.close()

data = bytes(all_bytes)
print('Total bytes received:', len(data))
print()

# Find all packets starting with AE A7
i = 0
pkt_num = 0
while i < len(data) - 1:
    if data[i] == 0xAE and data[i+1] == 0xA7:
        j = i + 2
        while j < len(data) - 1 and not (data[j] == 0xAE and data[j+1] == 0xA7):
            j += 1
        pkt = data[i:j]
        pkt_num += 1
        sep = ' '
        hex_str = pkt.hex(sep)
        # Try to decode if it looks like a measurement packet (27 bytes)
        note = ''
        if len(pkt) == 27 and pkt[4] == 0x85:
            dist_raw = pkt[7] * 256 + pkt[8]
            if dist_raw > 32767:
                dist_raw -= 65536
            note = '  -> dist = ' + str(round(dist_raw * 0.1, 1)) + ' m'
        print('Packet', pkt_num, '(' + str(len(pkt)) + ' bytes):', hex_str + note)
        i = j
    else:
        i += 1
