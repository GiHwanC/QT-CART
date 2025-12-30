from HX711 import HX711
import json
import time

DT_PIN = 5
SCK_PIN = 6

# --- 보정값 불러오기 ---
with open("hx711_calib.json", "r") as f:
    calib = json.load(f)

OFFSET = calib["offset"]
SCALE  = calib["scale"]

hx = HX711(DT_PIN, SCK_PIN)

# 🟡 영점 값 (처음엔 0)
TARE = 0

print("실시간 무게 보기 시작 (Ctrl + C 로 종료)")
print("👉 아무것도 올리지 말고, 2~3초 기다려 주세요 (자동 영점)")

time.sleep(2)

# ---- 🔥 전원 켤 때 자동 TARING ----
total = 0
samples = 20

for _ in range(samples):
    total += hx.read_raw()
    time.sleep(0.02)

raw_zero = total / samples
TARE = (raw_zero - OFFSET) / SCALE

print(f"✔ 영점 설정 완료 (TARE = {TARE:.2f} g)")

while True:
    try:
        total = 0
        samples = 10

        for _ in range(samples):
            total += hx.read_raw()
            time.sleep(0.02)

        raw = total / samples
        weight = (raw - OFFSET) / SCALE

        # ➕ 영점 적용
        weight = weight - TARE

        if weight < 0:
            weight = 0

        print(f"무게: {weight:.2f} g")

    except KeyboardInterrupt:
        print("\n종료")
        break
