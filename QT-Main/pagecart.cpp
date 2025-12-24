#include "pagecart.h"
#include "ui_pagecart.h"
#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>

PageCart::PageCart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    // ROS2 노드 초기화
    m_node = rclcpp::Node::make_shared("page_cart_node");
    m_cmdVelPub = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 1. 드라이버 인스턴스 생성
    m_uwbDriver = new UwbDriver();

    // 2. USB 포트 열기 (내부적으로 스레드가 돌며 데이터 파싱 시작)
    if (m_uwbDriver->openDualPorts("/dev/ttyUSB0", "/dev/ttyUSB1")) {
        qDebug() << "[PageCart] UWB Driver Connected to /dev/ttyUSB0";
    } else {
        qDebug() << "[PageCart] Failed to open UWB Driver! Check Connection.";
    }

    // 3. 타이머 설정 (50ms마다 데이터 확인 = 20Hz)
    m_uwbTimer = new QTimer(this);
    connect(m_uwbTimer, &QTimer::timeout, this, &PageCart::onUwbTimerTimeout);
    m_uwbTimer->start(50);

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
}

PageCart::~PageCart()
{
    // 메모리 정리 및 스레드 종료
    if (m_uwbTimer) {
        m_uwbTimer->stop();
        delete m_uwbTimer;
    }
    if (m_uwbDriver) {
        m_uwbDriver->closePorts(); // 스레드 join 및 포트 닫기
        delete m_uwbDriver;
    }
    delete ui;
}

// 타이머 핸들러: 드라이버에서 최신 거리값 조회 및 로봇 제어
void PageCart::onUwbTimerTimeout()
{
    // 화면이 보이지 않거나(다른 페이지 이동), 안내 모드 등일 때는 제어 중지
    // if (!this->isVisible()) {
    //     controlDualRobot(0.0, 0.0); // 안전을 위해 정지 명령
    //     return;
    // }

    if (m_uwbDriver) {
        float l = 0.0f;
        float r = 0.0f;

        // 드라이버 내부의 Mutex로 보호된 최신값 가져오기 (Non-blocking)
        m_uwbDriver->getDistances(l, r);

        m_distL = l;
        m_distR = r;

        controlDualRobot(m_distL, m_distR);

        qDebug() << "UWB Dist: L=" << l << " R=" << r;
    }
}

void PageCart::controlDualRobot(float l, float r)
{
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
