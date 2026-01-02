import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from sqlalchemy.orm import Session
import models, database
from read_weight import read_cart_weight

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

@app.post("/cart/tare")
def tare_cart():
    global EXPECTED_WEIGHT
    EXPECTED_WEIGHT = read_cart_weight()

    return {
        "message": "Cart tared",
        "expected_weight": EXPECTED_WEIGHT
    }

# 상품 등록
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
@app.get("/scan/{item_id}")
def scan_item(item_id: int, db: Session = Depends(get_db)):
    global EXPECTED_WEIGHT, CART_ITEMS

    item = db.query(models.Item).filter(models.Item.id == item_id).first()
    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")

    CART_ITEMS.append({
        "id": item.id,
        "name": item.name,
        "price": item.price,
        "weight": item.weight
    })

    EXPECTED_WEIGHT += item.weight

    real_weight = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real_weight)
    diff = abs(real_weight - EXPECTED_WEIGHT)

    return {
        "action": "add",
        "item": item.name,
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real_weight,
        "diff": diff,
        "movable": movable
    }

# ➖ 상품 제거 (UI)
@app.post("/cart/remove/{item_id}")
def remove_item(item_id: int):
    global EXPECTED_WEIGHT, CART_ITEMS

    for i, item in enumerate(CART_ITEMS):
        if item["id"] == item_id:
            removed_item = CART_ITEMS.pop(i)
            EXPECTED_WEIGHT -= removed_item["weight"]
            break
    else:
        raise HTTPException(status_code=404, detail="Item not in cart")

    if EXPECTED_WEIGHT < 0:
        EXPECTED_WEIGHT = 0.0

    real_weight = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real_weight)
    diff = abs(real_weight - EXPECTED_WEIGHT)

    return {
        "action": "remove",
        "item": removed_item["name"],
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real_weight,
        "diff": diff,
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

@app.get("/cart/check")
def check_cart_weight():
    real_weight = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real_weight)
    diff = abs(real_weight - EXPECTED_WEIGHT)

    return {
        "expected_weight": EXPECTED_WEIGHT,
        "real_weight": real_weight,
        "diff": diff,
        "movable": movable
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
