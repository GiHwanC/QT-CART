#include "pagecart.h"
#include "ui_pagecart.h"
#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>
#include <QStackedWidget>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QNetworkDatagram>

PageCart::PageCart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PageCart)
{  ui->setupUi(this);

    ui->tableCart->setStyleSheet(
        /* 전체 테이블 */
        "QTableWidget {"
        " background-color: #ffffff;"
        " gridline-color: #e0e0e0;"
        " font: 11pt \"Ria Sans\";"
        " border: 1px solid #dcdcdc;"
        " selection-color: black;"
        "}"

        /* 셀 */
        "QTableWidget::item {"
        " padding: 6px;"
        "}"


        /* 헤더 전체 */
        "QHeaderView {"
        " background-color: #f5f5f5;"
        "}"

        /* 헤더 셀 */
        "QHeaderView::section {"
        " background-color: #f5f5f5;"
        " color: #333;"
        " font: bold 11pt \"Ria Sans\";"
        " border: 1px solid #dcdcdc;"
        " padding: 6px;"
        "}"

        /* 헤더 아래 구분선 제거 */
        "QHeaderView::section:horizontal {"
        " border-top: 0px;"
        "}"

        /* 스크롤바 (세로) */
        "QScrollBar:vertical {"
        " width: 12px;"
        " background: #f0f0f0;"
        "}"

        "QScrollBar::handle:vertical {"
        " background: #c1c1c1;"
        " border-radius: 6px;"
        " min-height: 30px;"
        "}"

        "QScrollBar::handle:vertical:hover {"
        " background: #a8a8a8;"
        "}"

        "QScrollBar::add-line:vertical,"
        "QScrollBar::sub-line:vertical {"
        " height: 0px;"
        "}"

        "QTableWidget {"
        " background-color: #ffffff;"
        " border: none;"
        " font: 11pt \"Ria Sans\";"
        "}"

        "QTableWidget::item {"
        " border-bottom: 1px solid #eeeeee;"
        " padding: 12px;"
        "}"

        "QHeaderView::section {"
        " background-color: #ffffff;"
        " border: none;"
        " border-bottom: 2px solid #f0f0f0;"
        " font: bold 11pt \"Ria Sans\";"
        " padding: 10px;"
        "}"

        "QScrollBar:vertical {"
        " width: 8px;"
        " background: transparent;"
        "}"
        "QScrollBar::handle:vertical {"
        " background: #d0d0d0;"
        " border-radius: 4px;"
        "}"
        );

    ui->tableCart->setSelectionMode(QAbstractItemView::NoSelection);


    m_node = rclcpp::Node::make_shared("page_cart_udp_node");
    m_cmdVelPub = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    m_udpSocket = new QUdpSocket(this);

    // 포트 44444 바인딩 (모든 IP에서 오는 데이터 수신)
    if (m_udpSocket->bind(QHostAddress::Any, 44444)) {
        qDebug() << "[PageCart] UDP Socket Listening on Port 44444";
    } else {
        qDebug() << "[PageCart] Failed to bind UDP Port!";
    }

    // 데이터가 들어오면 processPendingDatagrams 실행하도록 연결
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &PageCart::processPendingDatagrams);

    // 바코드 및 UI 설정
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
    connect(ui->btnPay, SIGNAL(clicked()), this, SLOT(on_btnPay_clicked()));
}

PageCart::~PageCart()
{
    delete ui;
}

void PageCart::processPendingDatagrams()
{
    // 대기 중인 패킷이 있으면 모두 처리
    while (m_udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_udpSocket->receiveDatagram();

        // 받은 데이터: "L:1.52" or "R:1.20"
        QString data = QString::fromUtf8(datagram.data()).trimmed();

        bool updated = false;

        // 데이터 파싱
        if (data.startsWith("L:")) {
            m_distL = data.mid(2).toFloat(); // "L:" 제거하고 숫자로 변환
            updated = true;
        }
        else if (data.startsWith("R:")) {
            m_distR = data.mid(2).toFloat(); // "R:" 제거하고 숫자로 변환
            updated = true;
        }

        // 값이 갱신되었으면 로봇 제어
        if (updated) {
            qDebug() << "UDP Recv: " << data << " (L:" << m_distL << " R:" << m_distR << ")";
            controlDualRobot(m_distL, m_distR);
        }
    }
}

void PageCart::controlDualRobot(float l, float r)
{
    // if (!this->isVisible()) return;

    auto msg = geometry_msgs::msg::Twist();

    // 유효하지 않은 값이면 정지
    if (l <= 0.01 || r <= 0.01) {
        m_cmdVelPub->publish(msg);
        return;
    }

    // 1. 평균 거리 (전진 여부 판단)
    float avg_dist = (l + r) / 2.0;

    // 2. 회전 제어
    float diff = r - l;
    if (std::abs(diff) < 0.1) diff = 0.0;  // 떨림 방지 Deadzone

    float turn_gain = 2.0; // 회전 민감도
    msg.angular.z = diff * turn_gain;

    // 1.2m 이상 -> 전진
    if (avg_dist > 1.2) {
        msg.linear.x = 0.2;
    }
    // 0.6m 이내 -> 정지
    else if (avg_dist < 0.6) {
        msg.linear.x = 0.0;
    }
    // 그 사이 -> 제자리 회전만
    else {
        msg.linear.x = 0.0;
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

void PageCart::on_pushButton_clicked()
{
    emit goWelcome();
}

void PageCart::on_btnPay_clicked()
{
    emit goPay();
}
