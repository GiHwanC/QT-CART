from HX711 import HX711
import time
import json

DT_PIN = 23
SCK_PIN = 24

hx = HX711(DT_PIN, SCK_PIN)

print("HX711 보정 시작")

print("❶ 아무것도 안 올린 상태에서 기다리세요...")
time.sleep(2)

offset_sum = 0
N = 20

for _ in range(N):
    raw = hx.read_raw()
    offset_sum += raw
    time.sleep(0.1)

offset = offset_sum / N
print(f"OFFSET = {offset}")

known_weight = float(input("❷ 기준 무게(g)를 입력하세요: "))

weight_sum = 0
for _ in range(N):
    raw = hx.read_raw()
    weight_sum += raw
    time.sleep(0.1)

raw_with_weight = weight_sum / N

scale = (raw_with_weight - offset) / known_weight

print("\n보정 완료")
print(f"offset = {offset}")
print(f"scale  = {scale}")


calib_data = {
    "offset": offset,
    "scale": scale
}

with open("hx711_calib.json", "w") as f:
    json.dump(calib_data, f, indent=4)

print(" hx711_calib.json 저장 완료")
