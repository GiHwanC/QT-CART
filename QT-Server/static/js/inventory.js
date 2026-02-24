let currentItems = [];

function refreshAll() {
    fetchInventory();
    fetchSalesStats();
}

async function fetchInventory() {
    try {
        const res = await fetch('/items');
        if (res.ok) {
            currentItems = await res.json();
            renderTable('expiry'); 
        }
    } catch (e) { console.error(e); }
}

function renderTable(sortKey) {
    const tbody = document.getElementById('inventoryBody');
    
    if (sortKey === 'expiry') {
        currentItems.sort((a, b) => new Date(a.expiry_date) - new Date(b.expiry_date));
    } else if (sortKey === 'stock') {
        currentItems.sort((a, b) => a.stock - b.stock);
    }

    const today = new Date();
    today.setHours(0,0,0,0);

    let html = currentItems.map(item => {
        const expiryDate = new Date(item.expiry_date);
        const diffTime = expiryDate - today;
        const diffDays = Math.ceil(diffTime / (1000 * 60 * 60 * 24)); 

        let dateClass = 'expiry-good';
        let dateText = `D-${diffDays}`;

        if (diffDays < 0) {
            dateClass = 'expiry-expired';
            dateText = '만료됨';
        } else if (diffDays === 0) {
            dateClass = 'expiry-warning';
            dateText = '오늘 만료';
        } else if (diffDays <= 3) { 
            dateClass = 'expiry-warning';
            dateText = `${diffDays}일 남음`;
        }

        let stockHtml = '';
        if (item.stock === 0) {
            stockHtml = `<span class="stock-badge stock-low">부족 (${item.stock})</span>`;
        } else {
            stockHtml = `<span class="stock-badge stock-normal">${item.stock}개</span>`;
        }

        return `<tr>
            <td style="color:#94a3b8;">#${item.id}</td>
            <td style="font-weight: bold;">${item.name}</td>
            <td>${item.price.toLocaleString()}원</td>
            <td>
                <span class="badge ${dateClass}">${item.expiry_date} (${dateText})</span>
            </td>
            <td>${stockHtml}</td>
        </tr>`;
    }).join('');
    
    tbody.innerHTML = html || '<tr><td colspan="5" style="text-align:center; padding:30px; color:#64748b;">등록된 상품이 없습니다.</td></tr>';
}

let myChart = null;

// ----- 결제 정보 -----
async function fetchSalesStats() {
    try {
        const res = await fetch('/payments');
        if (!res.ok) return;
        const payments = await res.json();

        const today = new Date();
        const thirtyDaysAgo = new Date();
        thirtyDaysAgo.setDate(today.getDate() - 30);

        let monthTotal = 0;
        let monthCount = 0;
        const dailySales = {};

        for (let i = 0; i < 30; i++) {
            const d = new Date();
            d.setDate(today.getDate() - i);
            dailySales[d.toISOString().split('T')[0]] = 0;
        }

        payments.forEach(pay => {
            const payDate = new Date(pay.created_at);
            if (payDate >= thirtyDaysAgo) {
                monthTotal += pay.total_price;
                monthCount += 1;
                const dateStr = pay.created_at.split('T')[0];
                if (dailySales[dateStr] !== undefined) dailySales[dateStr] += pay.total_price;
            }
        });

        document.getElementById('monthTotal').innerText = monthTotal.toLocaleString() + '원';
        document.getElementById('monthCount').innerText = monthCount + '건';
        updateChart(dailySales);

    } catch (e) { console.error(e); }
}

function updateChart(dailyData) {
    const sortedDates = Object.keys(dailyData).sort();
    const dataValues = sortedDates.map(date => dailyData[date]);
    const labels = sortedDates.map(date => date.slice(5)); 

    const ctx = document.getElementById('salesChart').getContext('2d');
    if (myChart) myChart.destroy(); 

    myChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: '매출 (원)',
                data: dataValues,
                borderColor: '#8b5cf6',
                backgroundColor: 'rgba(139, 92, 246, 0.2)', 
                borderWidth: 2,
                fill: true,
                tension: 0.3
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { display: false } },
            scales: {
                y: { 
                    beginAtZero: true, 
                    grid: { borderDash: [5, 5], color: '#334155' }, 
                    ticks: { color: '#94a3b8' } 
                },
                x: { 
                    grid: { display: false },
                    ticks: { color: '#94a3b8' } 
                }
            }
        }
    });
}

refreshAll();