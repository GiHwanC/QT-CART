#include "pagepay.h"
#include "ui_pagepay.h"
#include <QGraphicsDropShadowEffect>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QPushButton>
#include <QPixmap>

static QString moneyKR(int v)
{
    return QString::number(v) + "원";
}

PagePay::PagePay(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PagePay)
{
    ui->setupUi(this);

    auto makeShadow = [](QObject *parent){
        auto *s = new QGraphicsDropShadowEffect(parent);
        s->setBlurRadius(18);
        s->setOffset(0, 4);
        s->setColor(QColor(0, 0, 0, 60));
        return s;
    };

    // 그림자
    if (ui->frame_2) ui->frame_2->setGraphicsEffect(makeShadow(ui->frame_2));
    if (ui->widget)  ui->widget->setGraphicsEffect(makeShadow(ui->widget));

    // ✅ 버튼 연결 (새 UI 기준)
    if (ui->btnBack) connect(ui->btnBack, &QPushButton::clicked, this, &PagePay::onBackPressed);
    if (ui->btnNext) connect(ui->btnNext, &QPushButton::clicked, this, &PagePay::onNextPressed);

    // 테이블 세팅
    setupUiTable();

    // 카드 아이콘(임시 이미지)
    if (ui->lblCardIcon) {
        QPixmap px(":/etc/card.png");
        if (!px.isNull()) {
            ui->lblCardIcon->setPixmap(px.scaled(180, 180, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            ui->lblCardIcon->setAlignment(Qt::AlignCenter);
        } else {
            ui->lblCardIcon->setText("CARD");
            ui->lblCardIcon->setAlignment(Qt::AlignCenter);
        }
    }

    // frame 스타일
    if (ui->frame_2) {
        ui->frame_2->setStyleSheet("QFrame { background: white; border-radius: 16px; }");
    }

    // 안내 문구 (너 UI가 lblCarGuide면 그대로, lblCardGuide면 그걸로)
    if (ui->lblCarGuide) {
        ui->lblCarGuide->setText("카드를 투입구에 넣어주세요");
    }
    // 만약 실제로 lblCardGuide라면 위 대신 아래를 쓰면 됨:
    // if (ui->lblCardGuide) ui->lblCardGuide->setText("카드를 투입구에 넣어주세요");

    // 처음엔 비어있으니 기본 표시
    refreshTableAndTotal();
}


PagePay::~PagePay()
{
    delete ui;
}

void PagePay::setupUiTable()
{
    ui->tablePayItems->horizontalHeader()->setVisible(false);
    ui->tablePayItems->verticalHeader()->setVisible(false);

    ui->tablePayItems->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tablePayItems->setSelectionMode(QAbstractItemView::NoSelection);
    ui->tablePayItems->setShowGrid(false);

    // ✅ 5컬럼: [물품][간격][갯수][간격][가격]
    ui->tablePayItems->setColumnCount(5);

    auto *hh = ui->tablePayItems->horizontalHeader();
    hh->setSectionResizeMode(0, QHeaderView::Stretch); // 물품은 남는 폭 다 먹기
    hh->setSectionResizeMode(1, QHeaderView::Fixed);   // 간격
    hh->setSectionResizeMode(2, QHeaderView::Fixed);   // 갯수
    hh->setSectionResizeMode(3, QHeaderView::Fixed);   // 간격
    hh->setSectionResizeMode(4, QHeaderView::Fixed);   // 가격

    // ✅ 800x480 최적값(기본)
    int gapW   = 12;  // 물품-갯수 / 갯수-가격 사이 간격
    int qtyW   = 60;  // "3개" 정도 충분
    int priceW = 90;  // "13000원" 정도 충분

    // ✅ 테이블 폭에 따라 살짝 자동 보정 (작아지면 더 줄임)
    int tableW = ui->tablePayItems->viewport()->width();
    if (tableW > 0) {
        // 물품 칼럼 최소폭을 남기기 위해, 너무 좁으면 qty/price/gap를 줄여줌
        int fixed = gapW*2 + qtyW + priceW;
        int nameMin = 160; // 물품명 최소로 확보(원하면 140~200 조절)

        if (tableW - fixed < nameMin) {
            // 줄일 여지가 있는 것들부터 줄임
            gapW   = 8;
            qtyW   = 52;
            priceW = 78;
        }
    }

    ui->tablePayItems->setColumnWidth(1, gapW);
    ui->tablePayItems->setColumnWidth(2, qtyW);
    ui->tablePayItems->setColumnWidth(3, gapW);
    ui->tablePayItems->setColumnWidth(4, priceW);

    // (옵션) 글자와 셀 padding 조금 줄여서 7인치에 더 꽉 차게
    ui->tablePayItems->setStyleSheet(
        "QTableWidget::item { padding-left: 4px; padding-right: 4px; }"
        );
}


void PagePay::setPayItems(const QVector<PayLine>& lines)
{
    m_lines = lines;
    refreshTableAndTotal();
}

void PagePay::refreshTableAndTotal()
{
    ui->tablePayItems->setRowCount(0);

    int totalPrice = 0;

    for (int i = 0; i < m_lines.size(); ++i) {
        const auto &it = m_lines[i];

        int row = ui->tablePayItems->rowCount();
        ui->tablePayItems->insertRow(row);
        ui->tablePayItems->setRowHeight(row, 40); // ✅ 7인치면 40~44 추천

        // (0) 물품
        auto *nameItem = new QTableWidgetItem(it.name);
        nameItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        ui->tablePayItems->setItem(row, 0, nameItem);

        // (2) 갯수
        auto *qtyItem = new QTableWidgetItem(QString("%1개").arg(it.qty));
        qtyItem->setTextAlignment(Qt::AlignCenter);
        ui->tablePayItems->setItem(row, 2, qtyItem);

        // (4) 가격
        int linePrice = it.unitPrice * it.qty;
        totalPrice += linePrice;

        auto *priceItem = new QTableWidgetItem(moneyKR(linePrice));
        priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->tablePayItems->setItem(row, 4, priceItem);
    }

    if (ui->lblTotalPriceValue)
        ui->lblTotalPriceValue->setText(moneyKR(totalPrice));

    ui->btnNext->setEnabled(!m_lines.isEmpty());
}


void PagePay::onBackPressed()
{
    emit backCartClicked(); // ✅ PageCart로
}

void PagePay::onNextPressed()
{
    emit creditCardClicked(); // ✅ pagepay_card로
}
