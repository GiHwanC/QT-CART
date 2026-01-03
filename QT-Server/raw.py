from HX711 import HX711
import time

hx = HX711(23, 24)

while True:
    print(hx.read_raw())
    time.sleep(0.2)
