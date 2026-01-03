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

static QString moneyKR(qint64 v)
{
    const QLocale loc(QLocale::Korean, QLocale::SouthKorea);
    return loc.toString(v) + "원";
}

static QString imageForName(const QString& name)
{
    if(name=="홈런볼 초코맛") return ":/cart_snack(1).jpg";
    if(name=="허니버터칩") return ":/cart_snack(2).png";
    if(name=="오레오 초콜릿크림") return ":/cart_snack(3).jpg";
    if(name=="빼빼로 아몬드") return ":/cart_snack(4).jpg";
    if(name=="빈츠") return ":/cart_snack(5).jpg";
    if(name=="예감 치즈그르탕") return ":/cart_snack(6).jpg";
    if(name=="오!감자 감자그라탕") return ":/cart_snack(7).jpg";
    if(name=="포카칩 오리지널") return ":/cart_snack(8).jpg";
    if(name=="아이폰 15 프로") return ":/cart_iphone.jpg";
    if(name=="핸드크림") return ":/cart_handcream.jpg";
    if(name=="크리스마스 퍼즐") return ":/cart_puzzle.png";

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

    connect(m_scanner, &BarcodeScanner::itemFetched, this, &PageCart::handleItemFetched);

    // ✅ 너 헤더에 맞게: itemFetched(item, cartWeight) 형태로 연결
    connect(m_scanner, &BarcodeScanner::fetchFailed, this, &PageCart::handleFetchFailed);

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
    initFixedItems();
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

     resetCart();
    // initFixedItems();
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
    connect(btnPlus,  &QPushButton::clicked, this, &PageCart::onPlusClicked);

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
    connect(btnMinus, &QPushButton::clicked, this, &PageCart::onMinusClicked);

    // (5) 가격(금액)
    QTableWidgetItem *priceItem = new QTableWidgetItem(moneyKR(0));
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
    connect(btnDelete,&QPushButton::clicked, this, &PageCart::onDeleteClicked);

    updateRowAmount(row);
}

// ----------------------------------------
// + 버튼
// ----------------------------------------
void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 2, btn);
    if (row < 0 || row >= m_items.size()) return;

    // 1. itemId 확인
    int itemId = m_items[row].id;
    if (itemId <= 0) {
        qDebug() << "Invalid Item ID on Row:" << row;
        return;
    }

    // 2. [수정] 서버 전송 (POST /cart/add/{id})
    // m_scanner를 거치지 않고 직접 요청하여 확실하게 서버 상태를 업데이트합니다.
    QUrl url(QString("%1/cart/add/%2").arg(SERVER_BASE_URL).arg(itemId));
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
    manager->post(req, QByteArray()); // 빈 바디 전송

    // 3. UI 즉시 반영 (Optimistic Update)
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    ui->tableCart->item(row, 3)->setText(QString::number(qty + 1));

    updateRowAmount(row);

    updateTotal();
}

// ----------------------------------------
// - 버튼
// ----------------------------------------
void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 4, btn);
    if (row < 0 || row >= m_items.size()) return;

    int qty = ui->tableCart->item(row, 3)->text().toInt();
    if (qty <= 0) return; // 이미 0이면 무시

    int itemId = m_items[row].id;
    if (itemId <= 0) return;

    // 1. [수정] 서버 전송 (POST /cart/remove/{id})
    QUrl url(QString("%1/cart/remove/%2").arg(SERVER_BASE_URL).arg(itemId));
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
    manager->post(req, QByteArray());

    // 2. UI 즉시 반영
    ui->tableCart->item(row, 3)->setText(QString::number(qty - 1));

    updateRowAmount(row);
    updateTotal();
}

// ----------------------------------------
// X 삭제 버튼
// ----------------------------------------
void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = findRowByButton(ui->tableCart, 6, btn);
    if (row < 0 || row >= m_items.size()) return;

    int itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();

    // 1. [수정] 서버 전송
    // 서버 DB와 동기화하기 위해 현재 수량만큼 remove 요청을 보냅니다.
    if (itemId > 0 && qty > 0) {
        QNetworkAccessManager *manager = new QNetworkAccessManager(this);
        // 매니저는 모든 요청 처리 후 나중에 해제되도록 부모 지정

        QUrl url(QString("%1/cart/remove/%2").arg(SERVER_BASE_URL).arg(itemId));
        QNetworkRequest req(url);
        req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

        for (int i = 0; i < qty; ++i) {
            QNetworkReply *reply = manager->post(req, QByteArray());
            connect(reply, &QNetworkReply::finished, reply, &QNetworkReply::deleteLater);
        }
        // manager는 적절한 시점에 해제되도록 connect하거나 멤버변수로 관리 권장 (여기서는 간략화)
        connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
    }

    // 2. UI 데이터 정리 및 행 삭제
    if (row < m_unitPrice.size()) m_unitPrice.removeAt(row);
    if (row < m_items.size())     m_items.removeAt(row);

    ui->tableCart->removeRow(row);

    updateTotal();
}

// ----------------------------------------
// row 금액 = 수량 * 단가
// ----------------------------------------
void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;

    int qty = ui->tableCart->item(row, 3)->text().toInt();
    int amount = m_unitPrice[row] * qty;

    ui->tableCart->item(row, 5)->setText(moneyKR(amount));
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
        ui->lblTotalPrice_2->setText(moneyKR(totalPrice));

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

    m_scanner->fetchItemDetails(code);
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
                // [수정] .toInt() 제거! (QString 그대로 전달)
                m_scanner->fetchItemDetails(m_barcodeData);
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
        int unit = static_cast<int>(std::lround(item.price));  // 1500.0 -> 1500
        addRowForItem(item.name, unit, 1);

        int newRow = ui->tableCart->rowCount() - 1;
        if (newRow >= 0 && newRow < m_items.size()){
            m_items[newRow].id = item.id;
            m_items[newRow].weight = item.weight;
        }
    } else {
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));

        // 서버가 준 최신 price 반영하고 싶으면 여기서도 갱신 가능
        if (rowFound < m_unitPrice.size())
            m_unitPrice[rowFound] = static_cast<int>(std::lround(item.price));

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

void PageCart::on_btnCheckout_clicked()
{
    requestCheckWeightBeforeRun();
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
    // 1) 서버 cart 초기화
    QUrl url(QString("%1/cart/reset").arg(SERVER_BASE_URL));
    QNetworkRequest req(url);

    auto *manager = new QNetworkAccessManager(this);
    connect(manager, &QNetworkAccessManager::finished,
            this, [manager](QNetworkReply *reply){
                qDebug() << "[RESET] response =" << reply->readAll();
                reply->deleteLater();
                manager->deleteLater();
            });
    manager->post(req, QByteArray());

    // 2) ✅ UI 테이블 완전 비우기
    ui->tableCart->setRowCount(0);

    // 3) ✅ 내부 데이터도 초기화
    m_unitPrice.clear();
    m_items.clear();
    m_expectedWeight = 0.0;

    // 4) 라벨 갱신
    updateTotal();
}

void PageCart::requestCheckWeightBeforeRun()
{
    // 1. 서버 URL 준비
    QUrl url(QString("%1/cart/check_weight").arg(SERVER_BASE_URL));
    QNetworkRequest req(url);

    auto *manager = new QNetworkAccessManager(this);

    connect(manager, &QNetworkAccessManager::finished,
            this, [this, manager](QNetworkReply *reply){

                reply->deleteLater();
                manager->deleteLater();

                // 2. 네트워크 에러 체크
                if (reply->error() != QNetworkReply::NoError) {
                    QMessageBox::critical(this, "통신 오류",
                                          "서버와 연결할 수 없습니다.\n" + reply->errorString());
                    return;
                }

                // 3. JSON 파싱
                QByteArray data = reply->readAll();
                QJsonDocument doc = QJsonDocument::fromJson(data);
                if (!doc.isObject()) {
                    QMessageBox::warning(this, "데이터 오류", "서버 응답이 올바르지 않습니다.");
                    return;
                }

                QJsonObject obj = doc.object();

                // 서버 파이썬 코드에서 5% 오차를 계산하여 보내준 movable 값
                bool movable = obj.value("movable").toBool();
                double expected = obj.value("expected_weight").toDouble();
                double real = obj.value("real_weight").toDouble();
                double diff = obj.value("diff").toDouble();

                // 4. 판단 및 로봇 제어
                if (movable) {
                    // [성공] 오차 범위 이내 -> 결제 페이지 이동 및 로봇 구동
                    sendRobotMode(1); // 1: 로봇 구동 모드
                    emit goPay();     // 결제 화면으로 전환
                } else {
                    // [실패] 무게 불일치 -> 이동 불가 안내
                    QString msg = QString("상품 무게가 일치하지 않습니다.\n\n"
                                          "예상 무게: %1 g\n"
                                          "실제 무게: %2 g\n"
                                          "차이: %3 g\n\n"
                                          "카트의 물건을 확인해주세요.")
                                      .arg(expected, 0, 'f', 1)
                                      .arg(real, 0, 'f', 1)
                                      .arg(diff, 0, 'f', 1);

                    QMessageBox::warning(this, "출발 불가", msg);

                    sendRobotMode(0); // 0: 로봇 정지/대기 모드
                }
            });

    manager->get(req); // GET 요청 전송
}

void PageCart::sendRobotMode(int mode)
{
    QUdpSocket socket;
    QByteArray data = QString("MODE:%1").arg(mode).toUtf8();
    socket.writeDatagram(data, QHostAddress("192.168.123.43"), 55555);
}

void PageCart::initFixedItems()
{
    struct P { int id; QString name; int price; int qty; };

    // ✅ 테스트용: 장바구니에 "담긴 상태"로 올림 (qty 1~3 등)
    QVector<P> items = {
        { 1,  "홈런볼 초코맛",     1700, 1 },
        { 2,  "허니버터칩",        1800, 2 },
        { 3,  "오레오 초콜릿크림",  2500, 1 },
        { 4,  "빼빼로 아몬드",     1500, 3 },
        { 5,  "빈츠",             2000, 1 },
        { 6,  "예감 치즈그르탕",   1600, 2 },
        { 7,  "오!감자 감자그라탕", 1700, 1 },
        { 8,  "포카칩 오리지널",   1600, 2 },
        { 9,  "아이폰 15 프로", 1500000, 1 },
        { 10, "핸드크림",          3000, 1 },
        { 11, "크리스마스 퍼즐",  10000, 1 }
    };

    ui->tableCart->setRowCount(0);
    m_unitPrice.clear();
    m_items.clear();

    for (const auto &p : items) {
        addRowForItem(p.name, p.price, p.qty);

        int row = ui->tableCart->rowCount() - 1;
        if (row >= 0 && row < m_items.size()) {
            m_items[row].id = p.id;   // ✅ (+/-) 서버 연동하려면 실제 DB id로 맞춰야 함
        }
    }

    updateTotal();
}

void PageCart::handleItemFetched(const Item &item)
{
    // 1. 기존 테이블에 동일한 상품이 있는지 확인
    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *nameItem = ui->tableCart->item(r, 1);
        if (nameItem && nameItem->text() == item.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        // [신규 추가]
        int unit = static_cast<int>(std::lround(item.price));
        addRowForItem(item.name, unit, 1);

        int newRow = ui->tableCart->rowCount() - 1;
        if (newRow >= 0 && newRow < m_items.size()){
            m_items[newRow].id = item.id;
            m_items[newRow].weight = item.weight;
        }
    } else {
        // [수량 증가]
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));

        // 필요하다면 가격 정보 업데이트
        if (rowFound < m_items.size()) {
            m_items[rowFound].id = item.id;
            m_items[rowFound].weight = item.weight;
        }
        updateRowAmount(rowFound);
    }

    // 2. 전체 금액 및 라벨 갱신
    updateTotal();
}

void PageCart::handleFetchFailed(const QString &err)
{
    qDebug() << "Barcode fetch failed:" << err;
    QMessageBox::warning(this, "스캔 실패", "상품 정보를 불러오지 못했습니다.\n" + err);
}
