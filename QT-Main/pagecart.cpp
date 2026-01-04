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
// static int findRowByButton(QTableWidget* table, int col, QPushButton* btn)
// {
//     if (!table || !btn) return -1;

//     for (int r = 0; r < table->rowCount(); ++r) {
//         QWidget *cell = table->cellWidget(r, col);
//         if (!cell) continue;

//         const auto buttons = cell->findChildren<QPushButton*>();
//         for (auto *b : buttons) {
//             if (b == btn) return r;
//         }
//     }
//     return -1;
// }

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
}

PageCart::~PageCart()
{
    delete ui;
}

// ----------------------------------------
// 행 추가: 이미지/상품명/+/수량/-/가격/X
// ----------------------------------------
void PageCart::addRowForItem(const QString& id, const QString& name, int unitPrice, int qty, double weight)
{
    int row = ui->tableCart->rowCount();
    ui->tableCart->insertRow(row);
    ui->tableCart->setRowHeight(row, 60);

    // 가격 저장
    m_unitPrice.append(unitPrice);

    // 아이템 정보 저장 (QString ID 사용)
    ItemInfo info;
    info.id = id;
    info.name = name;
    info.price = unitPrice;
    info.weight = weight;
    m_items.append(info);

    // --- 셀 위젯 헬퍼 ---
    auto makeCenterCell = [&](QWidget *child) -> QWidget* {
        QWidget *w = new QWidget(ui->tableCart);
        auto *lay = new QHBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
        lay->setAlignment(Qt::AlignCenter);
        lay->addWidget(child);
        return w;
    };
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
        img->setPixmap(px.scaled(44, 44, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    }
    ui->tableCart->setCellWidget(row, 0, makeCenterCell(img));

    // (1) 상품명
    ui->tableCart->setItem(row, 1, new QTableWidgetItem(name));

    // (2) + 버튼
    QPushButton *btnPlus = new QPushButton("+", ui->tableCart);
    btnPlus->setFixedSize(26, 30);
    btnPlus->setStyleSheet("QPushButton { background-color: rgb(37,99,235); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 2, makeCenterCell(btnPlus));
    connect(btnPlus, &QPushButton::clicked, this, &PageCart::onPlusClicked);

    // (3) 수량
    QTableWidgetItem *qtyItem = new QTableWidgetItem(QString::number(qty));
    qtyItem->setTextAlignment(Qt::AlignCenter);
    ui->tableCart->setItem(row, 3, qtyItem);

    // (4) - 버튼
    QPushButton *btnMinus = new QPushButton("-", ui->tableCart);
    btnMinus->setFixedSize(30, 30);
    btnMinus->setStyleSheet("QPushButton { background-color: rgb(154,153,150); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 4, makeCenterCell(btnMinus));
    connect(btnMinus, &QPushButton::clicked, this, &PageCart::onMinusClicked);

    // (5) 가격
    QTableWidgetItem *priceItem = new QTableWidgetItem(moneyKR(0));
    priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->tableCart->setItem(row, 5, priceItem);

    // (6) X 삭제
    QPushButton *btnDelete = new QPushButton("X", ui->tableCart);
    btnDelete->setFixedSize(30, 30);
    btnDelete->setStyleSheet("QPushButton { background-color: rgb(224,27,36); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 6, makeRightCell(btnDelete));
    connect(btnDelete, &QPushButton::clicked, this, &PageCart::onDeleteClicked);

    updateRowAmount(row);
}

// ----------------------------------------
// + 버튼
// ----------------------------------------
void PageCart::onPlusClicked()
{
    qDebug() << "========================================";
    qDebug() << "[ACTION] + 버튼 클릭됨";

    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) {
        qDebug() << "[ERROR] sender가 QPushButton이 아닙니다.";
        return;
    }

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) {
        qDebug() << "[ERROR] 유효하지 않은 Row 인덱스입니다. Row:" << row << " Items Size:" << m_items.size();
        return;
    }

    QString itemId = m_items[row].id;
    qDebug() << "[INFO] 선택된 아이템 ID:" << itemId << " 이름:" << m_items[row].name;

    if (itemId.isEmpty() || itemId == "0") { 
        qDebug() << "[ERROR] 유효하지 않은 ID:" << itemId;
        return; 
    }

    // 서버 전송 로그
    qDebug() << "[NETWORK] 서버로 수량 증가 요청 전송 (ID:" << itemId << ")";
    QUrl url(QString("%1/cart/add/%2").arg(SERVER_BASE_URL).arg(itemId));
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
    
    // 응답 확인용 연결 (필요시 주석 해제)
    connect(manager, &QNetworkAccessManager::finished, this, [](QNetworkReply* reply){
        qDebug() << "[NETWORK] +요청 응답:" << reply->readAll();
        reply->deleteLater();
    });

    manager->post(req, QByteArray());

    // UI 반영
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    int newQty = qty + 1;
    qDebug() << "[UI] 수량 변경:" << qty << "->" << newQty;
    
    ui->tableCart->item(row, 3)->setText(QString::number(newQty));
    updateRowAmount(row);
    updateTotal();
}

// ----------------------------------------
// - 버튼
// ----------------------------------------
void PageCart::onMinusClicked()
{
    qDebug() << "========================================";
    qDebug() << "[ACTION] - 버튼 클릭됨";

    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) {
        qDebug() << "[ERROR] Row 찾기 실패. Row:" << row;
        return;
    }

    int qty = ui->tableCart->item(row, 3)->text().toInt();
    qDebug() << "[INFO] 현재 수량:" << qty;

    if (qty <= 0) {
        qDebug() << "[INFO] 수량이 0이므로 더 이상 줄일 수 없습니다.";
        return;
    }

    QString itemId = m_items[row].id;
    qDebug() << "[INFO] 아이템 ID:" << itemId;

    // 서버 전송
    qDebug() << "[NETWORK] 서버로 수량 감소 요청 전송";
    QUrl url(QString("%1/cart/remove/%2").arg(SERVER_BASE_URL).arg(itemId));
    QNetworkRequest req(url);
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
    manager->post(req, QByteArray());

    // UI 반영
    int newQty = qty - 1;
    qDebug() << "[UI] 수량 변경:" << qty << "->" << newQty;
    ui->tableCart->item(row, 3)->setText(QString::number(newQty));

    updateRowAmount(row);
    updateTotal();
}

void PageCart::onDeleteClicked()
{
    qDebug() << "========================================";
    qDebug() << "[ACTION] X(삭제) 버튼 클릭됨";

    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) {
        qDebug() << "[ERROR] Row 찾기 실패. Row:" << row;
        return;
    }

    QString itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();

    qDebug() << "[INFO] 삭제할 아이템 ID:" << itemId << " 현재 수량:" << qty;

    // 1. UI 먼저 삭제 (안정성 확보)
    qDebug() << "[UI] UI에서 행 삭제 진행. Row:" << row;
    m_unitPrice.removeAt(row);
    m_items.removeAt(row);
    ui->tableCart->removeRow(row);
    updateTotal();

    // 2. 서버 요청 전송 (반복문 수정)
    if (!itemId.isEmpty() && qty > 0) {
        qDebug() << "[NETWORK] 서버로 삭제 요청(" << qty << "회) 전송 시작";

        QUrl url(QString("%1/cart/remove/%2").arg(SERVER_BASE_URL).arg(itemId));
        QNetworkRequest req(url);
        req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

        // [중요 수정] 반복문 안에서 각각의 매니저를 생성하여 독립적으로 처리
        // 이렇게 해야 하나가 끝나도 다른 요청에 영향을 주지 않습니다.
        for (int i = 0; i < qty; ++i) {
            QNetworkAccessManager *manager = new QNetworkAccessManager(this);
            
            // 응답을 받으면 매니저와 reply를 메모리에서 해제
            connect(manager, &QNetworkAccessManager::finished, 
                    manager, [manager](QNetworkReply* reply){
                // 로그 확인용 (필요시 주석 해제)
                // qDebug() << "[NETWORK] 삭제 응답:" << reply->readAll();
                reply->deleteLater();
                manager->deleteLater(); 
            });

            manager->post(req, QByteArray());
        }
    }
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

void PageCart::handleItemFetched(const Item &item)
{
    QString strId = item.id;
    
    qDebug() << "[SCAN] Handled Item -> ID:" << strId << " Name:" << item.name;

    int rowFound = -1;
    // 기존에 같은 상품이 있는지 이름으로 검색 (ID로 검색해도 됨)
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        // 이름 기준 매칭
        if (m_items[r].name == item.name) { 
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        // 새 행 추가 (ID를 문자열로 전달)
        int unit = static_cast<int>(std::lround(item.price));
        addRowForItem(strId, item.name, unit, 1, item.weight);
    } else {
        // 수량 증가
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));
        
        // ID 업데이트 (혹시 모르니)
        m_items[rowFound].id = strId;
        
        updateRowAmount(rowFound);
    }
    updateTotal();
}

void PageCart::handleFetchFailed(const QString &err)
{
    qDebug() << "Barcode fetch failed:" << err;
    QMessageBox::warning(this, "스캔 실패", "상품 정보를 불러오지 못했습니다.\n" + err);
}

int PageCart::getRowFromButton(QWidget *btn)
{
    qDebug() << "[DEBUG] getRowFromButton 호출됨. 버튼 주소:" << btn;

    if (!btn) {
        qDebug() << "[ERROR] 버튼 포인터가 NULL입니다.";
        return -1;
    }
    
    // 테이블의 모든 행을 순회하며 이 버튼이 어디 들어있는지 찾습니다.
    int rowCount = ui->tableCart->rowCount();
    for (int r = 0; r < rowCount; ++r) {
        // 버튼이 있을만한 컬럼: 2(+), 4(-), 6(X)
        int targetCols[] = {2, 4, 6};
        
        for (int c : targetCols) {
            QWidget *cellWidget = ui->tableCart->cellWidget(r, c);
            
            // cellWidget이 존재하고, 그 안에 우리 버튼이 들어있다면 (부모-자식 관계)
            if (cellWidget && (cellWidget == btn || cellWidget->isAncestorOf(btn))) {
                qDebug() << "[DEBUG] 버튼 찾음! Row:" << r << ", Col:" << c;
                return r; // 찾은 줄 번호 반환
            }
        }
    }

    qDebug() << "[ERROR] 테이블에서 해당 버튼을 찾을 수 없습니다.";
    return -1;
}