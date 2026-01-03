import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse 
from sqlalchemy.orm import Session
from typing import List
from datetime import datetime
import models, database
import dashboard
import logging
from read_weight import read_cart_weight

CART_ITEMS = []
EXPECTED_WEIGHT = 0.0

app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

# 대시보드 라우터 등록
app.include_router(dashboard.router)
app.mount("/static", StaticFiles(directory="static"), name="static")

def get_db():
    db = database.SessionLocal()
    try: yield db
    finally: db.close()

# 무게 검증 로직 (5% 오차)
def is_movable(expected, real):
    if expected <= 0: return True
    diff_ratio = abs(real - expected) / expected
    return diff_ratio <= 0.05

# --- 페이지 라우팅
@app.get("/dashboard")
async def dashboard_view(): return FileResponse("templates/dashboard.html")

@app.get("/inventory")
async def inventory_view(): return FileResponse("templates/inventory.html")

# 카트 무게 기준값 설정
@app.post("/cart/tare")
def tare_cart():
    global EXPECTED_WEIGHT
    EXPECTED_WEIGHT = read_cart_weight()
    return {"expected_weight": EXPECTED_WEIGHT}

@app.get("/items", response_model=List[models.ItemSchema])
def read_all_items(db: Session = Depends(get_db)):
    return db.query(models.Item).all()

# 상품 추가 
@app.post("/cart/add/{item_id}")
def add_item(item_id: int, db: Session = Depends(get_db)):
    global EXPECTED_WEIGHT, CART_ITEMS

    item = db.query(models.Item).filter(models.Item.id == item_id).first()
    if not item:
        raise HTTPException(404, "Item not found")

    CART_ITEMS.append({
        "id": item.id,
        "name": item.name,
        "price": item.price,
        "weight": item.weight
    })

    EXPECTED_WEIGHT += item.weight

    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)

    return {
        "action": "add",
        "id": item.id,          # <--- 추가됨
        "item": item.name,
        "price": item.price,    # <--- 추가됨 (DB 가격 전달)
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "diff": abs(real - EXPECTED_WEIGHT),
        "movable": movable
    }

# 상품 제거
@app.post("/cart/remove/{item_id}")
def remove_item(item_id: int):
    global EXPECTED_WEIGHT, CART_ITEMS

    for i, item in enumerate(CART_ITEMS):
        if item["id"] == item_id:
            removed = CART_ITEMS.pop(i)
            EXPECTED_WEIGHT -= removed["weight"]
            break
    else:
        raise HTTPException(404, "Item not in cart")

    EXPECTED_WEIGHT = max(EXPECTED_WEIGHT, 0)

    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)

    return {
        "action": "remove",
        "id": removed["id"],       # <--- 추가됨
        "item": removed["name"],
        "price": removed["price"], # <--- 추가됨
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "movable": movable
    }

# 장바구니 조회
@app.get("/cart")
def get_cart():
    total_price = sum(item["price"] for item in CART_ITEMS)

    return {
        "items": CART_ITEMS,
        "total_count": len(CART_ITEMS),
        "total_price": total_price,
        "expected_weight": EXPECTED_WEIGHT
    }

# 무게 체크
@app.get("/cart/check_weight")
def check_cart():
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)

    return {
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "diff": abs(real - EXPECTED_WEIGHT),
        "movable": movable
    }

# 대시보드 상태 조회
@app.get("/cart/status")
def get_cart_status_for_dashboard():
    global EXPECTED_WEIGHT, CART_ITEMS
    
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)
    
    system_status = "NORMAL" if movable else "WARNING_WEIGHT_MISMATCH"
    
    return {
        "real_weight": real,
        "expected_weight": EXPECTED_WEIGHT,
        "cart_items": CART_ITEMS,
        "total_price": sum(item["price"] for item in CART_ITEMS),
        "total_count": len(CART_ITEMS),
        "system_status": system_status,
        "movable": movable
    }

# 결제
@app.post("/cart/pay")
def pay_cart(db: Session = Depends(get_db)):
    global CART_ITEMS, EXPECTED_WEIGHT
    
    if not CART_ITEMS:
        raise HTTPException(status_code=400, detail="장바구니가 비어있습니다.")

    total_price = sum(item["price"] for item in CART_ITEMS)

    new_pay = models.Pay(
        total_price=total_price,
        created_at=datetime.now()
    )
    db.add(new_pay)
    db.commit()

    reset_cart() 

    return {"msg": "결제 완료", "amount": total_price}

# 결제 내역 조회 
@app.get("/payments", response_model=List[models.PaySchema])
def get_payment_history(db: Session = Depends(get_db)):
    return db.query(models.Pay).order_by(models.Pay.created_at.desc()).all()

# 카트 초기화
@app.post("/cart/reset")
def reset_cart():
    global EXPECTED_WEIGHT, CART_ITEMS
    EXPECTED_WEIGHT = 0.0
    CART_ITEMS.clear()
    return {"ok": True}

# 로그 필터
class EndpointFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return all(x not in msg for x in ["/bot/check", "/bot/report", "/cart/status"])

logging.getLogger("uvicorn.access").addFilter(EndpointFilter())

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)