#include "pagecart.h"
#include "ui_pagecart.h"

#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>

// UDP 통신 및 네트워크 데이터그램 헤더
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QNetworkDatagram>

PageCart::PageCart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    qDebug() << "[PageCart] Constructor Called";

    m_node = rclcpp::Node::make_shared("page_cart_udp_node");
    m_cmdVelPub = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    m_udpSocket = new QUdpSocket(this);

    // 5005번 포트 바인딩
    if (m_udpSocket->bind(QHostAddress::AnyIPv4, 5005)) {
        connect(m_udpSocket, &QUdpSocket::readyRead, this, &PageCart::readUwbUdp);
        qDebug() << "[PageCart] UDP Socket Bound on Port 5005. Waiting for 'L:x,R:y' data...";
    } else {
        qDebug() << "[PageCart] UDP Socket Bind Failed!";
    }

    // ==========================================
    // [기존 바코드 및 UI 설정]
    // ==========================================
    m_editBarcode = new QLineEdit(this);
    m_editBarcode->setVisible(false);
    m_editBarcode->setFocusPolicy(Qt::StrongFocus);
    m_editBarcode->setFocus();

    connect(m_editBarcode, SIGNAL(returnPressed()), this, SLOT(onBarcodeEntered()));

    qApp->installEventFilter(this);

    m_scanner = new BarcodeScanner(this);

    connect(m_scanner, &BarcodeScanner::itemFetched, this, &PageCart::handleItemFetched);
    connect(m_scanner, &BarcodeScanner::fetchFailed, this, &PageCart::handleFetchFailed);

    ui->tableCart->setColumnCount(6);
    initDummyItems();
    updateTotal();

    connect(ui->btnGuideMode, SIGNAL(clicked()), this, SLOT(on_btnGuideMode_clicked()));
}

PageCart::~PageCart()
{
    delete ui;
}

// ---------------------------------------------------------
// [UDP 데이터 수신 및 파싱 슬롯]
// 형식: "L:1.50,R:1.60"
// ---------------------------------------------------------
void PageCart::readUwbUdp()
{
    // 장바구니 페이지가 보일 때만 작동
    // if (!this->isVisible()) return;

    while (m_udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        QString str = QString::fromUtf8(datagram.data()).trimmed();

        // 1. 콤마(,)를 기준으로 분리 -> ["L:1.50", "R:1.60"]
        qDebug() << str;
        QStringList parts = str.split(",", Qt::SkipEmptyParts);
        bool updated = false;

        for (const QString &part : parts) {
            // L 데이터 파싱
            if (part.startsWith("L:")) {
                m_distL = part.mid(2).toFloat(); // "L:" 다음부터 숫자 변환
                updated = true;
            }
            // R 데이터 파싱
            else if (part.startsWith("R:")) {
                m_distR = part.mid(2).toFloat(); // "R:" 다음부터 숫자 변환
                updated = true;
            }
        }

        // 2. 데이터가 갱신되었으면 제어 함수 호출
        if (updated) {
            controlDualRobot(m_distL, m_distR);
        }
    }
}

// ---------------------------------------------------------
// [듀얼 앵커 기반 주행 로직]
// ---------------------------------------------------------
void PageCart::controlDualRobot(float l, float r)
{
    auto msg = geometry_msgs::msg::Twist();

    // 0. 안전장치: 센서 값이 0이거나 튄 무시
    if (l <= 0.01 || r <= 0.01)
        return;

    // 1. 평균 거리 (전진 여부 판단)
    float avg_dist = (l + r) / 2.0;

    // 2. 회전 제어 (P제어: 거리 차이 비례)
    // 오른쪽이 더 멀다(R > L) -> 사람이 왼쪽에 있다 -> 왼쪽 회전 (+z)
    // 왼쪽이 더 멀다(L > R) -> 사람이 오른쪽에 있다 -> 오른쪽 회전 (-z)

    float diff = r - l;

    // 데드밴드 (차이가 일정 미만이면 떨림 방지용으로 무시)
    if (std::abs(diff) > 0.05) diff = 0.0;  // 0.05m

    float turn_gain = 2.0; // 회전 민감도 (조절 필요)
    msg.angular.z = diff * turn_gain;

    // 1.2m 이상 멀어지면 전진
    if (avg_dist > 1.2) {
        msg.linear.x = 0.2; // 0.2 m/s 속도
    }
    // 0.6m 이내로 너무 가까우면 정지
    else if (avg_dist < 0.6) {
        msg.linear.x = 0.0;
        // msg.angular.z = 0.0;
    }

    // 0.6 ~ 1.2m 사이는 유지 구역 (제자리 회전만 허용)
    else {
        msg.linear.x = 0.0;
        // 회전값은 유지 (사람을 계속 바라봄)
    }

    m_cmdVelPub->publish(msg);
}

void PageCart::initDummyItems()
{
    QStringList names   = {"사과", "바나나", "우유"};
    QVector<int> prices = {3000, 1500, 2500};

    ui->tableCart->setRowCount(names.size());
    m_unitPrice = prices;

    for (int row = 0; row < names.size(); ++row) {
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(names[row]));
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("0"));
        ui->tableCart->setItem(row, 4, new QTableWidgetItem("0"));

        QPushButton *btnPlus = new QPushButton("+", this);
        ui->tableCart->setCellWidget(row, 2, btnPlus);
        connect(btnPlus, SIGNAL(clicked()), this, SLOT(onPlusClicked()));

        QPushButton *btnMinus = new QPushButton("-", this);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));

        QPushButton *btnDelete = new QPushButton("삭제", this);
        ui->tableCart->setCellWidget(row, 5, btnDelete);
        connect(btnDelete, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    }
}

void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;
    int row = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 2) == btn) {
            row = r; break;
        }
    }
    if (row < 0) return;
    QTableWidgetItem *qtyItem = ui->tableCart->item(row, 1);
    int qty = qtyItem->text().toInt();
    qty++;
    qtyItem->setText(QString::number(qty));
    updateRowAmount(row);
    updateTotal();
}

void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;
    int row = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 3) == btn) {
            row = r; break;
        }
    }
    if (row < 0) return;
    QTableWidgetItem *qtyItem = ui->tableCart->item(row, 1);
    int qty = qtyItem->text().toInt();
    if (qty > 0) qty--;
    qtyItem->setText(QString::number(qty));
    updateRowAmount(row);
    updateTotal();
}

void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;
    int row = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 5) == btn) {
            row = r; break;
        }
    }
    if (row < 0) return;
    if (row < m_unitPrice.size()) m_unitPrice.removeAt(row);
    ui->tableCart->removeRow(row);
    updateTotal();
}

void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;
    int qty = ui->tableCart->item(row, 1)->text().toInt();
    int amount = m_unitPrice[row] * qty;
    ui->tableCart->item(row, 4)->setText(QString::number(amount));
}

void PageCart::updateTotal()
{
    int total = 0;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *amt = ui->tableCart->item(r, 4);
        if (!amt) continue;
        total += amt->text().toInt();
    }
    ui->labelTotalPriceValue->setText(QString::number(total) + "원");
}

void PageCart::onBarcodeEntered()
{
    QString code = m_editBarcode->text().trimmed();
    m_editBarcode->clear();
    if (code.isEmpty()) return;
    qDebug() << "[PageCart] barcode entered =" << code;
    m_scanner->fetchItemDetails(code);
}

bool PageCart::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_F11) {
            QWidget *top = this->window();
            if (top->isFullScreen()) top->showNormal();
            else top->showFullScreen();
            return true;
        }
        else if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            if (!m_barcodeData.isEmpty()) {
                m_scanner->fetchItemDetails(m_barcodeData);
                m_barcodeData.clear();
            }
            return true;
        }
        else if (!keyEvent->text().isEmpty() && !(keyEvent->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier))) {
            m_barcodeData.append(keyEvent->text());
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

void PageCart::handleItemFetched(const Item &item)
{
    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *nameItem = ui->tableCart->item(r, 0);
        if (nameItem && nameItem->text() == item.name) {
            rowFound = r; break;
        }
    }

    if (rowFound == -1) {
        int row = ui->tableCart->rowCount();
        ui->tableCart->insertRow(row);
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(item.name));
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("1"));
        ui->tableCart->setItem(row, 4, new QTableWidgetItem(QString::number(item.price)));
        m_unitPrice.append(static_cast<int>(item.price));

        QPushButton *btnPlus    = new QPushButton("+", this);
        QPushButton *btnMinus   = new QPushButton("-", this);
        QPushButton *btnDelete = new QPushButton("삭제", this);
        ui->tableCart->setCellWidget(row, 2, btnPlus);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        ui->tableCart->setCellWidget(row, 5, btnDelete);

        connect(btnPlus,  SIGNAL(clicked()), this, SLOT(onPlusClicked()));
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));
        connect(btnDelete,SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    } else {
        QTableWidgetItem *qtyItem = ui->tableCart->item(rowFound, 1);
        int qty = qtyItem->text().toInt();
        qty++;
        qtyItem->setText(QString::number(qty));
        updateRowAmount(rowFound);
    }
    updateTotal();
}

void PageCart::handleFetchFailed(const QString &error)
{
    QMessageBox::critical(this, "상품 조회 실패", error);
}

void PageCart::on_btnGuideMode_clicked()
{
    emit guideModeClicked();
}
