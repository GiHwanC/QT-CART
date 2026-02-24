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
    return ""; 
}

PageCart::PageCart(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    // 그림자 효과
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

    if (ui->widget_2) ui->widget_2->setGraphicsEffect(makeShadow(ui->widget_2));
    if (ui->widget)   ui->widget->setGraphicsEffect(makeShadow(ui->widget));

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

    m_scanner = new BarcodeScanner(this);
    connect(m_scanner, &BarcodeScanner::itemFetched, this, &PageCart::handleItemFetched);
    connect(m_scanner, &BarcodeScanner::fetchFailed, this, &PageCart::handleFetchFailed);

    // 테이블 컬럼 설정
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

    if (ui->btnHome) {
        connect(ui->btnHome, &QPushButton::clicked, this, [this](){
            emit goWelcome();
        });
    }
}

PageCart::~PageCart()
{
    delete ui;
}

// MainWidget으로부터 API 설정 주입
void PageCart::setApiConfig(const QString &ip, int port)
{
    if (m_scanner) {
        m_scanner->setApiBaseUrl(ip, port);
    }
    resetCart();
}

// 카트 행 추가 
void PageCart::addRowForItem(const QString& id, const QString& name, int unitPrice, int qty, double weight)
{
    int row = ui->tableCart->rowCount();
    ui->tableCart->insertRow(row);
    ui->tableCart->setRowHeight(row, 60);

    m_unitPrice.append(unitPrice);

    ItemInfo info;
    info.id = id; 
    info.name = name;
    info.price = unitPrice;
    info.weight = weight;
    m_items.append(info);

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

    // 이미지
    QLabel *img = new QLabel(ui->tableCart);
    img->setFixedSize(44, 44);
    img->setAlignment(Qt::AlignCenter);
    QPixmap px(imageForName(name));
    if (!px.isNull()) {
        img->setPixmap(px.scaled(44, 44, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    }
    ui->tableCart->setCellWidget(row, 0, makeCenterCell(img));
    ui->tableCart->setItem(row, 1, new QTableWidgetItem(name));

    QPushButton *btnPlus = new QPushButton("+", ui->tableCart);
    btnPlus->setFixedSize(26, 30);
    btnPlus->setStyleSheet("QPushButton { background-color: rgb(37,99,235); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 2, makeCenterCell(btnPlus));
    connect(btnPlus, &QPushButton::clicked, this, &PageCart::onPlusClicked);

    QTableWidgetItem *qtyItem = new QTableWidgetItem(QString::number(qty));
    qtyItem->setTextAlignment(Qt::AlignCenter);
    ui->tableCart->setItem(row, 3, qtyItem);

    QPushButton *btnMinus = new QPushButton("-", ui->tableCart);
    btnMinus->setFixedSize(30, 30);
    btnMinus->setStyleSheet("QPushButton { background-color: rgb(154,153,150); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 4, makeCenterCell(btnMinus));
    connect(btnMinus, &QPushButton::clicked, this, &PageCart::onMinusClicked);

    QTableWidgetItem *priceItem = new QTableWidgetItem(moneyKR(0));
    priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->tableCart->setItem(row, 5, priceItem);

    // 삭제 버튼
    QPushButton *btnDelete = new QPushButton("X", ui->tableCart);
    btnDelete->setFixedSize(30, 30);
    btnDelete->setStyleSheet("QPushButton { background-color: rgb(224,27,36); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 6, makeRightCell(btnDelete));
    connect(btnDelete, &QPushButton::clicked, this, &PageCart::onDeleteClicked);

    updateRowAmount(row);
}

void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    QString itemId = m_items[row].id;
    m_scanner->fetchItemDetails(itemId);
}

void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    QString itemId = m_items[row].id;
    m_scanner->removeItem(itemId);

    int currentQty = ui->tableCart->item(row, 3)->text().toInt();
    int newQty = currentQty - 1;

    if (newQty <= 0) {
        m_unitPrice.removeAt(row);
        m_items.removeAt(row);
        ui->tableCart->removeRow(row);
    } else {
        ui->tableCart->item(row, 3)->setText(QString::number(newQty));
        updateRowAmount(row);
    }
    updateTotal();
}

void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    QString itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();

    for (int i = 0; i < qty; ++i) {
        m_scanner->removeItem(itemId);
    }

    m_unitPrice.removeAt(row);
    m_items.removeAt(row);
    ui->tableCart->removeRow(row);
    updateTotal();
}

void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    int amount = m_unitPrice[row] * qty;
    ui->tableCart->item(row, 5)->setText(moneyKR(amount));
}

void PageCart::updateTotal()
{
    int totalCount = 0;
    int totalPrice = 0;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        auto *qtyItem = ui->tableCart->item(r, 3);
        if (!qtyItem) continue;
        int qty = qtyItem->text().toInt();
        totalCount += qty;
        int unit = (r < m_unitPrice.size()) ? m_unitPrice[r] : 0;
        totalPrice += unit * qty;
    }

    if (ui->lblTotalPrice_2)
        ui->lblTotalPrice_2->setText(moneyKR(totalPrice));
    if (ui->lblCartTitle)
        ui->lblCartTitle->setText(QString("Cart (%1 products)").arg(totalCount));
}

void PageCart::onBarcodeEntered()
{
    QString code = m_editBarcode->text().trimmed();
    m_editBarcode->clear();
    if (code.isEmpty()) return;
    m_scanner->fetchItemDetails(code);
}

bool PageCart::eventFilter(QObject *obj, QEvent *event)
{
    Q_UNUSED(obj);
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            if (!m_barcodeData.isEmpty()) {
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

void PageCart::handleItemFetched(const Item &item)
{
    QString strId = item.id;

    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (m_items[r].id == strId || m_items[r].name == item.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        int unit = static_cast<int>(std::lround(item.price));
        addRowForItem(strId, item.name, unit, 1, item.weight);
    } else {
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));
        
        m_items[rowFound].id = strId;
        m_items[rowFound].weight = item.weight;
        
        updateRowAmount(rowFound);
    }
    updateTotal();
}

void PageCart::handleFetchFailed(const QString &err)
{
    qDebug() << "Barcode fetch failed:" << err;
    QMessageBox::warning(this, "스캔 실패", "상품 정보를 불러오지 못했습니다.\n" + err);
}

void PageCart::on_btnGuide_clicked() { emit guideModeClicked(); }

void PageCart::on_pushButton_clicked() { resetCart(); }

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
    ui->tableCart->setRowCount(0);
    m_unitPrice.clear();
    m_items.clear();
    updateTotal();
}

int PageCart::getRowFromButton(QWidget *btn)
{
    if (!btn) return -1;
    int rowCount = ui->tableCart->rowCount();
    for (int r = 0; r < rowCount; ++r) {
        int targetCols[] = {2, 4, 6};
        for (int c : targetCols) {
            QWidget *cellWidget = ui->tableCart->cellWidget(r, c);
            if (cellWidget && (cellWidget == btn || cellWidget->isAncestorOf(btn))) {
                return r;
            }
        }
    }
    return -1;
}

double PageCart::getTotalWeight() const
{
    double totalWeight = 0.0;
    // m_items와 테이블의 행은 동기화되어 있다고 가정
    for(int i = 0; i < m_items.size(); ++i) {
        if (i >= ui->tableCart->rowCount()) break;

        QTableWidgetItem *itemQty = ui->tableCart->item(i, 3);
        if (itemQty) {
            int qty = itemQty->text().toInt();
            totalWeight += (m_items[i].weight * qty);
        }
    }
    return totalWeight;
}

void PageCart::on_btnCheckout_clicked()
{
    if (ui->tableCart->rowCount() == 0) {
        QMessageBox::information(this, "알림", "장바구니가 비어있습니다.");
        return;
    }
    
    // [수정] 예상 무게를 계산하여 MainWidget으로 전달
    double expected = getTotalWeight();
    qDebug() << "Request Checkout - Expected Weight:" << expected;
    emit requestCheckout(expected); 
}