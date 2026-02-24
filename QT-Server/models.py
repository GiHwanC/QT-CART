from sqlalchemy import Column, Integer, String, Float, DateTime
from database import Base
from pydantic import BaseModel
from datetime import datetime

# 상품 테이블
class Item(Base):
    __tablename__ = "items"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True)
    price = Column(Float)
    stock = Column(Integer)
    weight = Column(Float)       # 무게 로직용
    expiry_date = Column(String) # 대시보드 유통기한 관리용

# 결제 내역 테이블 
class Pay(Base):
    __tablename__ = "payments"

    id = Column(Integer, primary_key=True, index=True)
    total_price = Column(Float)
    created_at = Column(DateTime, default=datetime.now)

# 상품 생성 요청 
class ItemCreate(BaseModel):
    name: str
    price: float
    stock: int
    weight: float
    expiry_date: str 

# 상품 조회 응답 
class ItemSchema(BaseModel):
    id: int
    name: str
    price: float
    stock: int
    weight: float
    expiry_date: str 
    
    class Config:
        from_attributes = True

# 결제 내역 응답 (대시보드 차트용)
class PaySchema(BaseModel):
    id: int
    total_price: float
    created_at: datetime
    
    class Config:
        from_attributes = True

class CartScanResponse(BaseModel):
    item: ItemSchema
    cart_weight: float
    expected_weight: float
    weight_ok: bool
    diff: float
    
    allowed: float

# 무게 검증 응답 
class CartScanResponse(BaseModel):
    item: ItemSchema        # 스캔된 상품 정보
    cart_weight: float      # 현재 실제 무게
    expected_weight: float  # 예상 무게
    weight_ok: bool         # 무게 일치 여부
    diff: float             # 오차
    allowed: float          # 허용 오차 범위

# 무게 포함 아이템 정보 
class ItemWithWeight(ItemSchema):
    cart_weight: float
