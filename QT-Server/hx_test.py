from HX711 import HX711
import time
import json

# ▼ json 있으면 불러오고, 없으면 기본값
try:
    with open("hx711_calib.json", "r") as f:
        calib = json.load(f)
        OFFSET = calib.get("offset", 0.0)
        SCALE  = calib.get("scale", 1.0)
except:
    OFFSET = 0.0
    SCALE  = 1.0

hx = HX711(16, 21)

def read_weight(n=15, delay=0.02):
    """샘플 여러 번 읽어서 평균"""
    vals = []

    for _ in range(n):
        v = hx.read_raw()
        vals.append(v)
        time.sleep(delay)

    vals.sort()

    # 앞뒤 노이즈 컷
    if len(vals) > 6:
        vals = vals[3:-3]

    raw = sum(vals) / len(vals)

    # 보정 적용
    weight = (raw - OFFSET) / SCALE

    if weight < 0:
        weight = 0

    return raw, round(weight, 2)


if __name__ == "__main__":
    print("=== HX711 TEST MODE ===")
    print("Ctrl + C 로 종료")

    while True:
        raw, w = read_weight()

        print(f"RAW={raw:.2f}   WEIGHT={w} g   (OFFSET={OFFSET}, SCALE={SCALE})")
        time.sleep(0.3)
