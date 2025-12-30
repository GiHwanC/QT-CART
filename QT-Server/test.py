from HX711 import HX711
import time

DT = 5
SCK = 6

hx = HX711(DT, SCK)

print("RAW TEST")
while True:
    try:
        v = hx.read_raw()
        print(v)
        time.sleep(0.3)
    except KeyboardInterrupt:
        break
