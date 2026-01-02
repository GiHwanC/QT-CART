import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse 
from sqlalchemy.orm import Session
from typing import List
import models, database
import logging
import dashboard
# from read_weight import read_cart_weight

CART_ITEMS = []
EXPECTED_WEIGHT = 0.0

app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

# 대시보드 라우터 등록
app.include_router(dashboard.router)
app.mount("/static", StaticFiles(directory="static"), name="static")

# 무게 측정 함수 
def read_cart_weight():
    return 300.0  

def get_db():
    db = database.SessionLocal()
    try: yield db
    finally: db.close()

# 무게 검증 로직 (5% 오차)
def is_movable(expected, real):
    if expected <= 0: return True
    diff_ratio = abs(real - expected) / expected
    return diff_ratio <= 0.05

# --- 페이지 라우팅 ---
@app.get("/dashboard")
async def dashboard_view(): return FileResponse("templates/dashboard.html")

@app.get("/inventory")
async def inventory_view(): return FileResponse("templates/inventory.html")

# --- 상품 및 장바구니 API ---
@app.post("/items/", response_model=models.ItemSchema)
def create_item(item: models.ItemCreate, db: Session = Depends(get_db)):
    db_item = models.Item(**item.dict())
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item

@app.get("/items", response_model=List[models.ItemSchema])
def read_all_items(db: Session = Depends(get_db)):
    return db.query(models.Item).all()

@app.post("/cart/add/{item_id}")
def add_item_to_cart(item_id: int, db: Session = Depends(get_db)):
    global EXPECTED_WEIGHT, CART_ITEMS
    item = db.query(models.Item).filter(models.Item.id == item_id).first()
    if not item: raise HTTPException(status_code=404, detail="Item not found")
    
    CART_ITEMS.append({"id": item.id, "name": item.name, "price": item.price, "weight": item.weight})
    EXPECTED_WEIGHT += item.weight
    
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)
    return {"msg": "Added", "movable": movable, "expected": EXPECTED_WEIGHT, "real": real}

@app.post("/cart/remove/{item_id}")
def remove_item_from_cart(item_id: int):
    global EXPECTED_WEIGHT, CART_ITEMS
    target_idx = next((i for i, item in enumerate(CART_ITEMS) if item["id"] == item_id), None)
    if target_idx is not None:
        removed = CART_ITEMS.pop(target_idx)
        EXPECTED_WEIGHT -= removed["weight"]
        if EXPECTED_WEIGHT < 0: EXPECTED_WEIGHT = 0.0
        return {"msg": "Removed", "expected": EXPECTED_WEIGHT}
    raise HTTPException(status_code=404, detail="Item not in cart")

@app.get("/cart/status")
def get_cart_full_status():
    global EXPECTED_WEIGHT, CART_ITEMS
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)
    status_text = "NORMAL" if movable else "WARNING_WEIGHT_MISMATCH"
    return {
        "real_weight": real,
        "expected_weight": EXPECTED_WEIGHT,
        "cart_items": CART_ITEMS,
        "total_price": sum(item["price"] for item in CART_ITEMS),
        "total_count": len(CART_ITEMS),
        "system_status": status_text
    }

@app.post("/cart/reset")
def reset_cart():
    global CART_ITEMS, EXPECTED_WEIGHT
    CART_ITEMS = []
    EXPECTED_WEIGHT = 0.0
    return {"msg": "Reset done"}

@app.on_event("startup")
def startup_event():
    reset_cart()

# --- 로그 필터 ---
class EndpointFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return all(x not in msg for x in ["/bot/check", "/bot/report", "/cart/status"])

logging.getLogger("uvicorn.access").addFilter(EndpointFilter())

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)