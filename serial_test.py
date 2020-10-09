import serial

import signal
import sys
import time
millis = int(round(time.time() * 1000))
try:
    # Initialize serial connection
    ser = serial.Serial('/dev/ttyACM1', 115200)
except Exception as e:
    print("Failed to establish a serial connection:")
    print(e)
    sys.exit(-1)

sisend = input("sisesta fps: ")
fps = 1000/ int(sisend)

while True:
    # Read the serial input to string
    if  int(round(time.time() * 1000)) - millis >= fps:
        ser.write(str.encode("{\"MS\":{\"M1\": 50,\"M2\": 50,\"M3\": 50}, \"LED\": [ 10066682, 10102972, 10319056, 10355969, 10428612, 10565919, 10717752, 10783710, 10882887, 10955120, 11127807, 11158075, 11193819, 11202752, 11551486, 11623994, 11790515, 11880182, 11931999, 11952266, 12133489, 12137829, 12293910, 12413209, 12442287, 12479550, 12521919, 12638736, 12678538, 12696715, 12873257, 13166316, 13348606, 13567725, 13850595, 13898195, 13924676, 13975870, 14025285, 14064364, 14141714, 14744475, 14744976, 15186068, 15224251, 15359457, 15409668, 15469622, 15520422, 15599931, 15727045, 15742858, 15788763, 15923926, 15978899, 16054705, 16173431, 16180072, 16431170]}\n")) 
        millis = int(round(time.time() * 1000))
