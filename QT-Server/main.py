import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from sqlalchemy.orm import Session
import models, database
from read_weight import read_cart_weight
from pydantic import BaseModel

class CartWeightCheck(BaseModel):
    expected_weight: float

EXPECTED_WEIGHT = 0.0
CART_ITEMS = []

app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

def get_db():
    db = database.SessionLocal()
    try:
        yield db
    finally:
        db.close()

def is_movable(expected, real):
    if expected <= 0:
        return True
    diff_ratio = abs(real - expected) / expected
    return diff_ratio <= 0.05

# 🧺 카트 무게 기준값 설정
@app.post("/cart/tare")
def tare_cart():
    global EXPECTED_WEIGHT
    EXPECTED_WEIGHT = read_cart_weight()
    return {"expected_weight": EXPECTED_WEIGHT}

# 📦 상품 등록
@app.post("/items/")
def create_item(item: models.ItemCreate, db: Session = Depends(get_db)):
    db_item = models.Item(
        name=item.name,
        price=item.price,
        stock=item.stock,
        weight=item.weight
    )
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item

# ➕ 상품 추가 (스캔)
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
        "item": item.name,
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "diff": abs(real - EXPECTED_WEIGHT),
        "movable": movable
    }

# ➖ 상품 제거
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
        "item": removed["name"],
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "movable": movable
    }

# 🛒 장바구니 조회
@app.get("/cart")
def get_cart():
    total_price = sum(item["price"] for item in CART_ITEMS)

    return {
        "items": CART_ITEMS,
        "total_count": len(CART_ITEMS),
        "total_price": total_price,
        "expected_weight": EXPECTED_WEIGHT
    }

# ⚖️ 무게 체크
@app.get("/cart/check_weight")
def check_cart():
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)

    return {
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real,
        "diff": abs(real - EXPECTED_WEIGHT),
        "movable": movable,
        "stop_type": "abnormal" if not movable else "none"
    }

# 🔄 카트 초기화
@app.post("/cart/reset")
def reset_cart():
    global EXPECTED_WEIGHT, CART_ITEMS
    EXPECTED_WEIGHT = 0.0
    CART_ITEMS.clear()
    return {"ok": True}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
