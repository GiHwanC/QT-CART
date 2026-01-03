from hx711 import HX711
import time, json

DT_PIN = 5
SCK_PIN = 6

hx = HX711(DT_PIN, SCK_PIN)

print("HX711 보정 시작")
time.sleep(2)

# 1) 빈 상태
print("1️⃣ 아무것도 안 올리고 기다리세요...")
N = 30
empty = sum(hx.get_value(5) for _ in range(N)) / N
print("OFFSET =", empty)

known = float(input("2️⃣ 기준 무게(g): "))

print("3️⃣ 기준 무게 올리고 기다리세요...")

raw = sum(hx.get_value(5) for _ in range(N)) / N

scale = (raw - empty) / known

print("\n보정 완료")
print("offset =", empty)
print("scale  =", scale)

calib = {
    "offset": empty,
    "scale": scale
}

with open("hx711_calib.json", "w") as f:
    json.dump(calib, f, indent=4)

print("✔ hx711_calib.json 저장 완료")
