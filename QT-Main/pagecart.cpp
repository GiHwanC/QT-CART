#include "pagecart.h"
#include "ui_pagecart.h"

#include <QGraphicsDropShadowEffect>
#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <cmath>


#define SERVER_BASE_URL "http://192.168.123.43:8000"

static QString imageForName(const QString& name)
{
    if (name == "헬로키티 인형")   return ":/etc/kitty.jpg";
    if (name == "바나나")         return ":/item/banana.jpg";
    if (name == "불닭볶음면 컵")   return ":/item/fire.jpg";
    return ""; // 기본값
}

// ------------------------------
// wrapper 셀 안의 버튼으로 row 찾기
// ------------------------------
static int findRowByButton(QTableWidget* table, int col, QPushButton* btn)
{
    if (!table || !btn) return -1;

    for (int r = 0; r < table->rowCount(); ++r) {
        QWidget *cell = table->cellWidget(r, col);
        if (!cell) continue;

        const auto buttons = cell->findChildren<QPushButton*>();
        for (auto *b : buttons) {
            if (b == btn) return r;
        }
    }
    return -1;
}

PageCart::PageCart(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    // 그림자
    auto makeShadow = [](QObject *parent){
        auto *s = new QGraphicsDropShadowEffect(parent);
        s->setBlurRadius(18);
        s->setOffset(0, 4);
        s->setColor(QColor(0, 0, 0, 60));
        return s;
    };
    ui->tableCart->setStyleSheet(
        "QTableWidget::item { padding: 0px; margin: 0px; }"
        "QTableWidget { border: none; }"
        );

    // ui에 widget / widget_2 가 존재 (새 UI)
    if (ui->widget_2) ui->widget_2->setGraphicsEffect(makeShadow(ui->widget_2));
    if (ui->widget)   ui->widget->setGraphicsEffect(makeShadow(ui->widget));

    // 테이블 기본s
    ui->tableCart->setSelectionMode(QAbstractItemView::NoSelection);
    ui->tableCart->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableCart->setShowGrid(false);

    // 바코드 입력용 숨김 QLineEdit
    m_editBarcode = new QLineEdit(this);
    m_editBarcode->setVisible(false);
    m_editBarcode->setFocusPolicy(Qt::StrongFocus);
    m_editBarcode->setFocus();
    connect(m_editBarcode, SIGNAL(returnPressed()), this, SLOT(onBarcodeEntered()));

    qApp->installEventFilter(this);

    // 서버 연동 BarcodeScanner
    m_scanner = new BarcodeScanner(this);

    connect(m_scanner, &BarcodeScanner::cartResult, this, &PageCart::onCartResult);

    // ✅ 7컬럼: [이미지][상품명][+][수량][-][가격][X]
    ui->tableCart->setColumnCount(7);
    ui->tableCart->horizontalHeader()->setVisible(false);
    ui->tableCart->verticalHeader()->setVisible(false);

    ui->tableCart->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->tableCart->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

    ui->tableCart->setColumnWidth(0, 54);
    ui->tableCart->setColumnWidth(2, 52);
    ui->tableCart->setColumnWidth(3, 42);
    ui->tableCart->setColumnWidth(5, 76);
    ui->tableCart->setColumnWidth(6, 46);

    updateTotal();

    // 새 UI 버튼들
    if (ui->btnHome) {
        connect(ui->btnHome, &QPushButton::clicked, this, [this](){
            emit goWelcome();
        });
    }
    if (ui->btnGuide) {
        connect(ui->btnGuide, SIGNAL(clicked()), this, SLOT(on_btnGuide_clicked()));
    }
    if (ui->btnCheckout) {
        connect(ui->btnCheckout, SIGNAL(clicked()), this, SLOT(on_btnCheckout_clicked()));
    }

    // clear cart 버튼(pushButton)
    if (ui->pushButton) {
        connect(ui->pushButton, &QPushButton::clicked, this, &PageCart::resetCart);
    }
}

PageCart::~PageCart()
{
    delete ui;
}

// ----------------------------------------
// 행 추가: 이미지/상품명/+/수량/-/가격/X
// ----------------------------------------
void PageCart::addRowForItem(const QString& name, int unitPrice, int qty)
{
    int row = ui->tableCart->rowCount();
    ui->tableCart->insertRow(row);
    ui->tableCart->setRowHeight(row, 60);

    // unit price 저장
    m_unitPrice.append(unitPrice);

    // weight 관리용 info도 같이 맞춰 넣어둠(서버 아이템 없을 때는 0)
    ItemInfo info;
    info.id = -1; 
    info.name = name;
    info.price = unitPrice;
    info.weight = 0.0;
    m_items.append(info);

    // 가운데 정렬 wrapper
    auto makeCenterCell = [&](QWidget *child) -> QWidget* {
        QWidget *w = new QWidget(ui->tableCart);
        auto *lay = new QHBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
        lay->setAlignment(Qt::AlignCenter);
        lay->addWidget(child);
        return w;
    };

    // 오른쪽 정렬 wrapper (X 버튼용)
    auto makeRightCell = [&](QWidget *child) -> QWidget* {
        QWidget *w = new QWidget(ui->tableCart);
        auto *lay = new QHBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
        lay->addStretch();
        lay->addWidget(child);
        return w;
    };

    // (0) 이미지
    QLabel *img = new QLabel(ui->tableCart);
    img->setFixedSize(44, 44);
    img->setAlignment(Qt::AlignCenter);

    QPixmap px(imageForName(name));
    if (!px.isNull()) {
        img->setPixmap(
            px.scaled(
                44, 44,
                Qt::KeepAspectRatioByExpanding,
                Qt::SmoothTransformation
            )
        );
    }

    ui->tableCart->setCellWidget(row, 0, makeCenterCell(img));

    // (1) 상품명
    ui->tableCart->setItem(row, 1, new QTableWidgetItem(name));

    // (2) + 버튼
    QPushButton *btnPlus = new QPushButton("+", ui->tableCart);
    btnPlus->setFixedSize(26, 30);
    btnPlus->setStyleSheet(
        "QPushButton {"
        " background-color: rgb(37, 99, 235);"
        " color: rgb(255, 255, 255);"
        " border: none;"
        " border-radius: 10px;"
        " font-weight: 700;"
        "}"
        "QPushButton:hover { background-color: rgb(29, 78, 216); }"
        "QPushButton:pressed { background-color: rgb(30, 64, 175); }"
        );
    ui->tableCart->setCellWidget(row, 2, makeCenterCell(btnPlus));
    connect(btnPlus, SIGNAL(clicked()), this, SLOT(onPlusClicked()));

    // (3) 수량
    QTableWidgetItem *qtyItem = new QTableWidgetItem(QString::number(qty));
    qtyItem->setTextAlignment(Qt::AlignCenter);
    ui->tableCart->setItem(row, 3, qtyItem);

    // (4) - 버튼
    QPushButton *btnMinus = new QPushButton("-", ui->tableCart);
    btnMinus->setFixedSize(30, 30);
    btnMinus->setStyleSheet(
        "QPushButton {"
        " background-color: rgb(154, 153, 150);"
        " color: rgb(255, 255, 255);"
        " border: none;"
        " border-radius: 10px;"
        " font-weight: 700;"
        "}"
        "QPushButton:hover { background-color: rgb(120, 120, 120); }"
        "QPushButton:pressed { background-color: rgb(94, 92, 100); }"
        );
    ui->tableCart->setCellWidget(row, 4, makeCenterCell(btnMinus));
    connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));

    // (5) 가격(금액)
    QTableWidgetItem *priceItem = new QTableWidgetItem("0");
    priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->tableCart->setItem(row, 5, priceItem);

    // (6) X 삭제
    QPushButton *btnDelete = new QPushButton("X", ui->tableCart);
    btnDelete->setFixedSize(30, 30);
    btnDelete->setStyleSheet(
        "QPushButton {"
        " background-color: rgb(224, 27, 36);"
        " color: rgb(255, 255, 255);"
        " border: none;"
        " border-radius: 10px;"
        " font-weight: 800;"
        "}"
        "QPushButton:hover { background-color: rgb(192, 28, 40); }"
        "QPushButton:pressed { background-color: rgb(150, 0, 15); }"
        );
    ui->tableCart->setCellWidget(row, 6, makeRightCell(btnDelete));
    connect(btnDelete, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));

    updateRowAmount(row);
}

// ----------------------------------------
// + 버튼
// ----------------------------------------
void PageCart::onPlusClicked() {
    auto *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 2, btn);
    if (row < 0 || row >= m_items.size()) return;

    int itemId = m_items[row].id;
    if (itemId <= 0) return;

    // ✅ UI 변경 금지. 서버 요청만.
    m_scanner->addItem(itemId);
}

// ----------------------------------------
// - 버튼
// ----------------------------------------
void PageCart::onMinusClicked() {
    auto *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 4, btn);
    if (row < 0 || row >= m_items.size()) return;

    int itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    if (itemId <= 0 || qty <= 0) return;

    // ✅ UI 변경 금지. 서버 요청만.
    m_scanner->removeItem(itemId);
}

// ----------------------------------------
// X 삭제 버튼
// ----------------------------------------
void PageCart::onDeleteClicked() {
    auto *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 6, btn);
    if (row < 0 || row >= m_items.size()) return;

    int itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    if (itemId <= 0 || qty <= 0) return;

    // ✅ 아직 진행중 제거가 있으면 막거나, 덮어쓰기 정책 택1
    if (m_pendingRemove.has_value()) return;

    m_pendingRemove = PendingRemove{ itemId, qty };

    // 첫 remove 요청 시작 (UI 변경 X)
    m_scanner->removeItem(itemId);
}

// ----------------------------------------
// row 금액 = 수량 * 단가
// ----------------------------------------
void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;

    int qty = ui->tableCart->item(row, 3)->text().toInt();
    int amount = m_unitPrice[row] * qty;

    ui->tableCart->item(row, 5)->setText(QString::number(amount));
}

// ----------------------------------------
// 전체 총액 / 총수량 / products 카운트 갱신
// - 새 UI 기준: lblTotalPrice_2 에 "총수량" 표시(네 기존 코드 유지)
// - lblCartTitle 에 "Cart (n products)" 표시
// ----------------------------------------
void PageCart::updateTotal()
{
    int totalCount = 0;     // 장바구니 전체 수량 합
    int totalPrice = 0;     // 총 금액 합

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        auto *qtyItem = ui->tableCart->item(r, 3);
        if (!qtyItem) continue;

        int qty = qtyItem->text().toInt();
        totalCount += qty;

        // ✅ 총 금액 누적(단가 * 수량)
        int unit = (r < m_unitPrice.size()) ? m_unitPrice[r] : 0;
        totalPrice += unit * qty;
    }

    // ✅ (A) "총 금액" 표시 (너가 말한 lblTotalPrice_2)
    if (ui->lblTotalPrice_2)
        ui->lblTotalPrice_2->setText(QString::number(totalPrice) + "원");



    // ✅ Cart (n products)
    if (ui->lblCartTitle)
        ui->lblCartTitle->setText(QString("Cart (%1 products)").arg(totalCount));
}

// ----------------------------------------
// 바코드 엔터(숨김 QLineEdit)
// ----------------------------------------
void PageCart::onBarcodeEntered()
{
    QString code = m_editBarcode->text().trimmed();
    m_editBarcode->clear();
    if (code.isEmpty()) return;

    m_scanner->addItem(code.toInt());
}

// ----------------------------------------
// 키 이벤트(바코드 누적)
// ----------------------------------------
bool PageCart::eventFilter(QObject *obj, QEvent *event)
{
    Q_UNUSED(obj);

    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            if (!m_barcodeData.isEmpty()) {
                m_scanner->addItem(m_barcodeData.toInt());
                m_barcodeData.clear();
            }
            return true;
        }
        else if (!keyEvent->text().isEmpty() &&
                 !(keyEvent->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier))) {
            m_barcodeData.append(keyEvent->text());
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

// ----------------------------------------
// 장바구니 반영(테이블)
// ----------------------------------------
void PageCart::addItemByScan(const Item &item)
{
    int rowFound = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *nameItem = ui->tableCart->item(r, 1);
        if (nameItem && nameItem->text() == item.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        addRowForItem(item.name, static_cast<int>(item.price), 1);

        // 방금 추가된 row에 weight 업데이트
        int newRow = ui->tableCart->rowCount() - 1;
        if (newRow >= 0 && newRow < m_items.size()){
            m_items[newRow].id = item.id;
            m_items[newRow].weight = item.weight;
        }
    } else {
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));

        // 같은 상품이면 weight 갱신(0일 때만 채워도 되고, 그냥 덮어써도 됨)
        if (rowFound < m_items.size()) {
            m_items[rowFound].id = item.id; 
            m_items[rowFound].weight = item.weight;
        }

        updateRowAmount(rowFound);
    }
}

// ----------------------------------------
// UI 버튼 슬롯들 (필요한 것만)
// ----------------------------------------
void PageCart::on_btnGuide_clicked()
{
    emit guideModeClicked();
}

void PageCart::on_pushButton_clicked()
{
    // clear cart 버튼이 auto-connection으로 여기 들어올 수도 있음
    resetCart();
}

void PageCart::checkWeightBeforeMove()
{
    if (m_checkingWeight) return;
    m_checkingWeight = true;
    m_scanner->checkCart();
}

void PageCart::on_btnCheckout_clicked()
{
    // 새 UI는 btnCheckout 쓰니까 비워도 됨
    checkWeightBeforeMove();
}

// ----------------------------------------
// 결제 페이지에서 쓰려고 만든 API들
// ----------------------------------------
QVector<PageCart::CartLine> PageCart::getCartLines() const
{
    QVector<CartLine> out;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        auto *nameItem = ui->tableCart->item(r, 1);
        auto *qtyItem  = ui->tableCart->item(r, 3);
        if (!nameItem || !qtyItem) continue;

        int qty = qtyItem->text().toInt();
        if (qty <= 0) continue;

        int unitPrice = (r < m_unitPrice.size()) ? m_unitPrice[r] : 0;
        out.push_back({ nameItem->text(), qty, unitPrice });
    }
    return out;
}

void PageCart::resetCart()
{
    // ✅ UI 변경 금지. 서버 요청만.
    m_scanner->resetCart();
}

void PageCart::sendRobotMode(int mode)
{
    QUdpSocket socket;
    QByteArray data = QString("MODE:%1").arg(mode).toUtf8();
    socket.writeDatagram(data, QHostAddress("192.168.123.43"), 55555);
}

void PageCart::onCartResult(const CartServerResult& r)
{
    m_checkingWeight = false;

    if (!r.ok) {
        QMessageBox::warning(this, "서버 오류", r.error);
        return;
    }

    // expectedWeight는 서버 값으로 동기화
    m_expectedWeight = r.expectedWeight;

    // movable false면 즉시 STOP
    if (!r.movable) {
        if (!m_isStopped) {
            m_isStopped = true;
            sendRobotMode(0);
            QMessageBox::warning(this, "무게 이상", "무게 불일치로 로봇 정지");
        }
        return; // ✅ UI 반영하지 않는 정책이면 여기서 종료
    }

    // ✅ 여기서부터 UI 반영 가능

    if (r.action == CartAction::AddOne) {
        addItemByScan(r.item);      // 네가 이미 갖고 있음(수량 증가/행 추가)
        updateTotal();
        return;
    }

    if (r.action == CartAction::RemoveOne) {
        applyRemoveOneToUi(r.itemId);
        updateTotal();

        // ✅ X삭제 진행 중이면 다음 remove 계속
        if (m_pendingRemove.has_value() && m_pendingRemove->itemId == r.itemId) {
            m_pendingRemove->remaining--;
            if (m_pendingRemove->remaining > 0) {
                m_scanner->removeItem(r.itemId); // 다음 제거 계속
            } else {
                m_pendingRemove.reset();
            }
        }
        return;
    }

    if (r.action == CartAction::CheckOnly) {

        // 서버 판단 실패 → 출발 불가
        if (!r.movable) {
            QMessageBox::warning(
                this,
                "출발 불가",
                "카트 무게가 일치하지 않습니다.\n출발할 수 없습니다."
            );
            sendRobotMode(0);
            return;
        }

        // 통과 → 출발
        QMessageBox::information(
            this,
            "출발",
            "무게 확인 완료. 출발합니다."
        );
        m_isStopped = false;
        sendRobotMode(1);
        return;
    }

    if (r.action == CartAction::Reset) {
        // reset 응답 성공 후 UI 완전 초기화
        ui->tableCart->setRowCount(0);
        m_items.clear();
        m_unitPrice.clear();
        m_expectedWeight = 0.0;
        m_isStopped = false;
        updateTotal();
        return;
    }
}


void PageCart::applyRemoveOneToUi(int itemId)
{
    // itemId로 row 찾기
    int rowFound = -1;
    for (int r = 0; r < m_items.size(); ++r) {
        if (m_items[r].id == itemId) { rowFound = r; break; }
    }
    if (rowFound < 0) return;

    int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
    qty -= 1;

    if (qty <= 0) {
        // row 제거
        if (rowFound < m_unitPrice.size()) m_unitPrice.removeAt(rowFound);
        if (rowFound < m_items.size())     m_items.removeAt(rowFound);
        ui->tableCart->removeRow(rowFound);
    } else {
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty));
        updateRowAmount(rowFound);
    }
}
