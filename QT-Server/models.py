from sqlalchemy import Column, Integer, String, Float
from database import Base
from pydantic import BaseModel

class Item(Base):
    __tablename__ = "items"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True)
    price = Column(Float)
    stock = Column(Integer)
    weight = Column(Float)

class ItemCreate(BaseModel):
    name: str
    price: float
    stock: int
    weight: float

class ItemSchema(BaseModel):
    id: int
    name: str
    price: float
    stock: int
    weight: float

    class Config:
        from_attributes = True
