import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import models, database

from read_weight import read_cart_weight


app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

app = FastAPI()

def get_db():
    db = database.SessionLocal()
    try:
        yield db
    finally:
        db.close()

# 상품 등록
@app.post("/items/", response_model=models.ItemSchema)
def create_item(item: models.ItemCreate, db: Session = Depends(get_db)):
    db_item = models.Item(name=item.name, price=item.price, stock=item.stock, weight=item.weight)
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item

# 상품 조회 (ID/바코드)
@app.get("/items/{item_id}", response_model=models.ItemSchema)
def read_item(item_id: str, db: Session = Depends(get_db)):
    try:
        r_id = int(item_id)
        item = db.query(models.Item).filter(models.Item.id == r_id).first()
    except:
        item = None
        
    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")
    return item

# 전체 상품 조회
@app.get("/items", response_model=List[models.ItemSchema])
def read_all_items(db: Session = Depends(get_db)):
    return db.query(models.Item).all()

if __name__ == "__main__":
    print("Starting API Server on Port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
@app.get("/cart/weight")
def get_cart_weight():
    return {
        "cart_weight": read_cart_weight(),
        "unit": "g"
    }

    
