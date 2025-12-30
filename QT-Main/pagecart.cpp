#include "pagecart.h"
#include "ui_pagecart.h"
#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>
#include <QStackedWidget>
#include <cmath> 

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

    connect(m_scanner, &BarcodeScanner::requestStop,
        this, [this](){
            qDebug() << "[PageCart] Weight mismatch → STOP";
            sendRobotMode(0);
        });

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

void PageCart::addItemByScan(const Item &item)
{
    int rowFound = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->item(r, 0)->text() == item.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound >= 0) {
        // 기존 상품 → 수량 +1
        int qty = ui->tableCart->item(rowFound, 1)->text().toInt() + 1;
        ui->tableCart->item(rowFound, 1)->setText(QString::number(qty));
        updateRowAmount(rowFound);
    } else {
        // 신규 상품
        int row = ui->tableCart->rowCount();
        ui->tableCart->insertRow(row);

        ui->tableCart->setItem(row, 0, new QTableWidgetItem(item.name));
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("1"));
        ui->tableCart->setItem(row, 4,
            new QTableWidgetItem(QString::number(item.price)));

        m_unitPrice.append(item.price);

        ItemInfo info;
        info.name   = item.name;
        info.price  = item.price;
        info.weight = item.weight;
        m_items.append(info);

        QPushButton *btnPlus   = new QPushButton("+", this);
        QPushButton *btnMinus  = new QPushButton("-", this);
        QPushButton *btnDelete = new QPushButton("삭제", this);

        ui->tableCart->setCellWidget(row, 2, btnPlus);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        ui->tableCart->setCellWidget(row, 5, btnDelete);

        connect(btnPlus,  &QPushButton::clicked, this, &PageCart::onPlusClicked);
        connect(btnMinus, &QPushButton::clicked, this, &PageCart::onMinusClicked);
        connect(btnDelete,&QPushButton::clicked, this, &PageCart::onDeleteClicked);
    }
}

void PageCart::updateExpectedWeightByScan(double itemWeight)
{
    m_expectedWeight += itemWeight;
}

bool PageCart::checkWeightOrStop(double cartWeight)
{
    double diff = std::fabs(cartWeight - m_expectedWeight);

    qDebug() << "[WEIGHT CHECK]"
             << "expected =" << m_expectedWeight
             << "real =" << cartWeight
             << "diff =" << diff;

    if (diff > 30.0) {
        qDebug() << "[STOP] 무게 불일치";
        sendRobotMode(0);
        return false;
    }
    return true;
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

    double w = m_items[row].weight;
    m_expectedWeight += w; 

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

    double w = m_items[row].weight;
    m_expectedWeight -= w;

    if (m_expectedWeight < 0)
        m_expectedWeight = 0;
    
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
    int qty = ui->tableCart->item(row, 1)->text().toInt();
    double w = m_items[row].weight * qty;

    m_expectedWeight -= w;
    if (m_expectedWeight < 0)
        m_expectedWeight = 0;

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

void PageCart::sendRobotMode(int mode)
{
    qDebug() << "[SEND ROBOT MODE]" << mode;

    // TODO: 실제 UDP / Serial 제어
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

void PageCart::handleItemFetched(const Item &item, double cartWeight)
{
    addItemByScan(item);                       // 수량 처리
    updateExpectedWeightByScan(item.weight);   // 무게 +1

    if (!checkWeightOrStop(cartWeight))
        return;

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
