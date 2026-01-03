from hx711 import HX711
import json, time

DT_PIN = 5
SCK_PIN = 6

with open("hx711_calib.json") as f:
    calib = json.load(f)

OFFSET = calib["offset"]
SCALE  = calib["scale"]

hx = HX711(DT_PIN, SCK_PIN)

hx.set_offset(OFFSET)
hx.set_reference_unit(SCALE)

print(" 무게 측정 시작")

while True:
    weight = hx.get_weight(10)
    print(f"무게: {weight:.1f} g")
    time.sleep(0.5)
