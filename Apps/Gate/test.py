import sys
import time
import struct

from serial import Serial

s = Serial(sys.argv[1], baudrate=1000000)
t0 = time.time()

while True:
    data = s.read(64)

    # i = data.find(42)
    # if i != 0:
    #     s.read(i)
    #     print('******** Need to recaler', i)
    #     continue
    # print(struct.unpack(b'BBBBB', data[:5]), int((time.time() - t0) * 1000))
    # t0 = time.time()

    (sensor_type, value_type, nb_data) = struct.unpack(b'BBB', data[:3])

    msg = [sensor_type, value_type, nb_data]

    for i in range(nb_data):
        start = 3 + i * 6
        end = 3 + (i + 1) * 6

        id, err, pos = struct.unpack(b'<BBf', data[start:end])
        msg += [id, err, pos]

    print(msg)
