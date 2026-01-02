from fastapi import APIRouter
from pydantic import BaseModel
from datetime import datetime

router = APIRouter(
    prefix="/dashboard",
    tags=["Dashboard"]
)

bot_state = {
    "status": "OFFLINE",
    "last_updated": datetime.now(),
    "battery": 0.0,
    "speed": 0.0
}

BOT_TIMEOUT_SEC = 5  # 5초간 보고 없으면 오프라인

class BotReport(BaseModel):
    status: str
    battery: float = 0.0
    speed: float = 0.0

class BotConnectionInfo(BaseModel):
    bot_status: str
    battery: float
    speed: float
    last_update: str

# 로봇 -> 서버
@router.post("/bot/report")
def report_bot_status(report: BotReport):
    global bot_state
    bot_state["status"] = report.status
    bot_state["last_updated"] = datetime.now()
    bot_state["battery"] = report.battery
    bot_state["speed"] = report.speed
    return {"msg": "status report received"}

# 서버 -> 프론트엔드
@router.get("/bot/check", response_model=BotConnectionInfo)
def check_bot_connection():
    global bot_state
    
    elapsed = (datetime.now() - bot_state["last_updated"]).total_seconds()
    
    current_status = bot_state["status"]
    if elapsed > BOT_TIMEOUT_SEC:
        current_status = "OFFLINE"
        bot_state["speed"] = 0.0
    
    return {
        "bot_status": current_status,
        "battery": bot_state["battery"],
        "speed": bot_state["speed"],
        "last_update": bot_state["last_updated"].strftime("%H:%M:%S")
    }

# 서버 연결 확인
@router.get("/health")
def health_check():
    return {"status": "ok", "time": datetime.now().isoformat()}