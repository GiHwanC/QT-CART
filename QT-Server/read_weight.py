from HX711 import HX711
import json
import threading
import time

# --- 보정값 로드 ---
with open("hx711_calib.json", "r") as f:
    calib = json.load(f)

OFFSET = calib["offset"]
SCALE  = calib["scale"]

hx = HX711(16, 21)

hx_lock = threading.Lock()

def read_cart_weight():
    """현재 카트 무게(g)를 반환"""
    with hx_lock:
        # 여러 번 평균 내서 노이즈 줄이기
        samples = 10
        total = 0

        for _ in range(samples):
            total += hx.read_raw()
            time.sleep(0.02)

        raw = total / samples
        weight = (raw - OFFSET) / SCALE

        # 음수/미세값 보호
        if weight < 0:
            weight = 0

        return round(weight, 2)
