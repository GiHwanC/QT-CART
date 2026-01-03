from HX711 import HX711
import json
import threading
import time
import sys

# --- 1. 보정값 로드 ---
try:
    with open("hx711_calib.json", "r") as f:
        calib = json.load(f)
    OFFSET = calib["offset"]
    SCALE  = calib["scale"]
    print(f"보정값 로드 완료: OFFSET={OFFSET}, SCALE={SCALE}")
except FileNotFoundError:
    print("오류: 'hx711_calib.json' 파일을 찾을 수 없습니다.")
    sys.exit(1)

# --- 2. HX711 초기화 (설정 함수 제거) ---
# 에러가 발생했던 set_reading_format, set_reference_unit 제거함
hx = HX711(23, 24)

# 만약 reset() 함수도 없다면 아래 줄도 주석 처리하거나 지우세요.
# hx.reset() 

hx_lock = threading.Lock()

def read_cart_weight():
    """현재 카트 무게(g)를 반환"""
    with hx_lock:
        # 여러 번 평균 내서 노이즈 줄이기
        samples = 10
        total = 0

        for _ in range(samples):
            # 님께서 원래 쓰시던 함수 그대로 사용
            total += hx.read_raw()
            time.sleep(0.02)

        raw = total / samples
        
        # 수동 계산 (라이브러리 설정 대신 직접 계산)
        weight = (raw - OFFSET) / SCALE

        # 음수/미세값 보호
        if weight < 0:
            weight = 0

        return round(weight, 2)

# --- 3. 실행 테스트 (0.5초 간격) ---
if __name__ == "__main__":
    print("--- 무게 측정 테스트 시작 (종료: Ctrl+C) ---")
    
    try:
        while True:
            # 무게 읽기
            current_weight = read_cart_weight()
            
            # 출력
            print(f"현재 무게: {current_weight} g")
            
            # 대기 (측정 시간 고려하여 0.3초 대기 -> 총 약 0.5초 주기)
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n--- 측정 종료 ---")
        
    except Exception as e:
        print(f"\n오류 발생: {e}")