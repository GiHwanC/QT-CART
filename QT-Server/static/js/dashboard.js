let currentCartId = 1;

// 1호차: 실제 터틀봇 
// 2~4호차: 시뮬레이션용 
const allCartsData = {
    1: { id: 1, name: "QTCART 1호차", isOnline: false, battery: 0, speed: 0, weight: 0, expected: 0, items: [], status: "OFFLINE" },
    2: { id: 2, name: "QTCART 2호차", isOnline: true, battery: 92, speed: 0.5, weight: 710, expected: 700, items: generateMockItems(3), status: "ONLINE" },
    3: { id: 3, name: "QTCART 3호차", isOnline: true, battery: 34, speed: 0.0, weight: 0, expected: 0, items: [], status: "ONLINE" },
    4: { id: 4, name: "QTCART 4호차", isOnline: true, battery: 65, speed: 0, weight: 0, expected: 0, items: [], status: "ONLINE" } 
};

// 가짜 아이템 생성 함수 (2~4호차용)
function generateMockItems(count) {
    const mockDb = [
        {name: "코카콜라 500ml", price: 2000, weight: 500},
        {name: "포카칩 오리지널", price: 1500, weight: 100},
        {name: "제주 삼다수 2L", price: 1000, weight: 2000},
        {name: "신라면 멀티팩", price: 4500, weight: 600}
    ];
    let items = [];
    for(let i=0; i<count; i++) {
        const rand = mockDb[Math.floor(Math.random() * mockDb.length)];
        items.push({...rand, id: 100+i});
    }
    return items;
}

// --- 메인 업데이트 루프 ---
async function updateDashboard() {
    const apiBadge = document.getElementById('apiBadge');

    // 실제 데이터 연동 1호차 로직
    try {
        // 1. ROS 봇 상태 및 배터리/속도 확인 
        const botRes = await fetch('/dashboard/bot/check');
        const botData = await botRes.json();
        
        // 2. 장바구니/무게 데이터 확인
        let cartData = null;
        try {
            const cRes = await fetch('/cart/status');
            if(cRes.ok) cartData = await cRes.json();
        } catch(e) {}

        // 서버 연결됨 표시
        apiBadge.className = 'status-badge on';
        apiBadge.innerHTML = '<span class="dot"></span> 서버 연결됨';

        const c1 = allCartsData[1]; 

        if(botData.bot_status === 'ONLINE') {
            c1.isOnline = true;
            c1.status = "ONLINE";
            
            // 배터리 & 속도
            c1.battery = parseInt(botData.battery);
            c1.speed = Math.abs(Math.round(botData.speed * 100));
            
            if(cartData) {
                // 무게 & 아이템
                c1.weight = cartData.real_weight;
                c1.expected = cartData.expected_weight;
                c1.items = cartData.cart_items || [];
                
                // 경고 상태 (5% 오차 로직 반영됨)
                if(cartData.system_status === "WARNING_WEIGHT_MISMATCH") {
                    c1.status = "WARNING";
                } else {
                    c1.status = "ONLINE";
                }
            }
        } else {
            // 오프라인일 때
            c1.isOnline = false;
            c1.status = "OFFLINE";
            c1.speed = 0;
        }

    } catch (err) {
        // 서버 에러 시 처리
        apiBadge.className = 'status-badge off';
        apiBadge.innerHTML = '<span class="dot"></span> 서버 끊김';
        
        allCartsData[1].isOnline = false;
        allCartsData[1].status = "OFFLINE";
    }

    // 1호차 이외 가짜 데이터 시뮬레이션
    updateMockData(2);
    updateMockData(3);
    updateMockData(4);

    // 화면 렌더링
    renderGrid();        // 상단 4개 카드
    renderDetailPanel(); // 하단 상세 정보
}

// 가짜 데이터 랜덤 업데이트 함수
function updateMockData(id) {
    const c = allCartsData[id];
    if(!c.isOnline) return;
    
    // 배터리 랜덤 감소
    if(Math.random() > 0.98) c.battery = Math.max(0, c.battery - 1);
    if(Math.random() > 0.7) c.speed = Math.floor(Math.random() * 120);
    else c.speed = 0.0;
}

// 상단 그리드 그리기
function renderGrid() {
    const grid = document.getElementById('cartGrid');
    grid.innerHTML = '';

    for(let i=1; i<=4; i++) {
        const data = allCartsData[i];
        const isSelected = (i === currentCartId) ? 'selected' : '';
        
        // 상태 점 색상 클래스 및 텍스트 색상 설정
        let dotClass = 'offline';
        let statusColor = '#64748b'; 

        if(data.isOnline) {
            if(data.status === 'WARNING') {
                dotClass = 'warning'; 
                statusColor = '#fbbf24'; 
            } else {
                dotClass = 'online';
                statusColor = '#34d399'; 
            }
        }
        
        // 속도 표시 로직
        const speedDisplay = data.isOnline ? `${data.speed} cm/s` : '--';

        const html = `
        <div class="cart-summary-card ${isSelected}" onclick="selectCart(${i})">
            <div class="summary-header">
                <div class="summary-name">${data.name}</div>
                <div class="summary-status-dot ${dotClass}" style="${dotClass === 'warning' ? 'background-color:#f97316; box-shadow:0 0 5px #f97316;' : ''}"></div>
            </div>
            <div class="summary-row" style="margin-top:5px;">
                <span>배터리</span>
                <span class="summary-val" style="color:${statusColor}">${data.isOnline ? data.battery+'%' : '-'}</span>
            </div>
            <div class="summary-row">
                <span>현재 속도</span> <span class="summary-val">${speedDisplay}</span> </div>
        </div>`;
        grid.innerHTML += html;
    }
}

// 하단 상세 패널 그리기
function renderDetailPanel() {
    const data = allCartsData[currentCartId]; // 현재 선택된 카트 데이터 가져오기
    const panel = document.getElementById('detailPanel');
    const statusText = document.getElementById('detailStatusText');
    
    // 텍스트 및 테두리 색상 설정
    document.getElementById('detailName').innerText = data.name;

    if(data.isOnline) {
        if(data.status === 'WARNING') {
            statusText.innerText = '● 무게 불일치 경고';
            statusText.style.color = '#fbbf24';
            panel.style.borderColor = '#fbbf24';
        } else {
            statusText.innerText = '● 온라인';
            statusText.style.color = '#34d399';
            panel.style.borderColor = '#10b981';
        }
    } else {
        statusText.innerText = '● 오프라인';
        statusText.style.color = '#f87171';
        panel.style.borderColor = '#ef4444';
    }

    // 센서 값 바인딩
    const elReal = document.getElementById('detailReal');
    const elExp = document.getElementById('detailExpected');
    const elBat = document.getElementById('detailBattery');
    const elSpd = document.getElementById('detailSpeed');

    if(data.isOnline) {
        elReal.innerText = typeof data.weight === 'number' ? data.weight.toFixed(1) : data.weight;
        elExp.innerText = typeof data.expected === 'number' ? data.expected.toFixed(1) : data.expected;
        elBat.innerText = data.battery;
        elSpd.innerText = data.speed;

        // 값에 따른 색상 클래스 부여
        elBat.parentElement.className = data.battery < 20 ? "data-value val-warning" : "data-value val-success";
        
        if(data.status === 'WARNING') elReal.parentElement.className = "data-value val-warning";
        else elReal.parentElement.className = "data-value val-normal";

    } else {
        elReal.innerText = '--';
        elExp.innerText = '--';
        elBat.innerText = '--';
        elSpd.innerText = '--';
        
        elReal.parentElement.className = "data-value val-offline";
        elBat.parentElement.className = "data-value val-offline";
    }

    // 아이템 리스트 렌더링
    const tbody = document.getElementById('detailListBody');
    const countEl = document.getElementById('detailCount');
    const priceEl = document.getElementById('detailPrice');
    
    if(!data.isOnline || data.items.length === 0) {
        tbody.innerHTML = '<tr><td colspan="4" style="text-align: center; color: #64748b; padding: 30px;">데이터 없음</td></tr>';
        countEl.innerText = '0';
        priceEl.innerText = '0원';
    } else {
        const agg = {};
        let totalP = 0;
        let totalC = 0;
        
        data.items.forEach(item => {
            if(agg[item.name]) agg[item.name].qty++;
            else agg[item.name] = {...item, qty: 1};
            totalP += item.price;
            totalC++;
        });

        let html = '';
        Object.values(agg).forEach(item => {
            html += `<tr>
                <td style="font-weight:600;">${item.name}</td>
                <td style="text-align: right;">${item.price.toLocaleString()}원</td>
                <td style="text-align: right;">${item.weight}g</td>
                <td style="text-align: right; font-weight: bold; color: #38bdf8;">${item.qty}</td>
            </tr>`;
        });
        tbody.innerHTML = html;
        countEl.innerText = totalC;
        priceEl.innerText = totalP.toLocaleString() + '원';
    }
}

// 카트 선택 함수 
function selectCart(id) {
    currentCartId = id;
    renderGrid();       // 선택된 스타일 적용을 위해 그리드 다시 그림
    renderDetailPanel(); // 하단 패널 내용 교체
}

// 1초마다 업데이트 실행
setInterval(updateDashboard, 1000);
updateDashboard();