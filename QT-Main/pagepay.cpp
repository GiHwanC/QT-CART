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
    ui->tablePayItems->horizontalHeader()->setVisible(false); // 위 헤더 숨김
    ui->tablePayItems->verticalHeader()->setVisible(false);   // 왼쪽 행번호 숨김

    ui->tablePayItems->setColumnCount(3);
    ui->tablePayItems->setHorizontalHeaderLabels({"물품", "갯수", "가격"});

    ui->tablePayItems->verticalHeader()->setVisible(false);
    ui->tablePayItems->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tablePayItems->setSelectionMode(QAbstractItemView::NoSelection);
    ui->tablePayItems->setShowGrid(false);

    ui->tablePayItems->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->tablePayItems->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    ui->tablePayItems->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

    ui->tablePayItems->setColumnWidth(1, 80);
    ui->tablePayItems->setColumnWidth(2, 140);
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
        ui->tablePayItems->setRowHeight(row, 44);

        // 물품>>>>>>> 5652d81 (gui 여동)
        auto *nameItem = new QTableWidgetItem(it.name);
        nameItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        ui->tablePayItems->setItem(row, 0, nameItem);

        // 갯수
        auto *qtyItem = new QTableWidgetItem(QString("%1개").arg(it.qty));
        qtyItem->setTextAlignment(Qt::AlignCenter);
        ui->tablePayItems->setItem(row, 1, qtyItem);

        // 가격
        int linePrice = it.unitPrice * it.qty;
        totalPrice += linePrice;

        auto *priceItem = new QTableWidgetItem(moneyKR(linePrice));
        priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->tablePayItems->setItem(row, 2, priceItem);
    }

    if (ui->lblTotalPriceValue) {
        ui->lblTotalPriceValue->setText(moneyKR(totalPrice));
    }

    // 카트 비었으면 다음 버튼 비활성
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
