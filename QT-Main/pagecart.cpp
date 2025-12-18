#include "pagecart.h"
#include "ui_pagecart.h"

#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>
#include <QStackedWidget>
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
        );



    // 1) 바코드 입력용 숨겨진 QLineEdit (GUI에는 안 보임)
    m_editBarcode = new QLineEdit(this);
    m_editBarcode->setVisible(false);
    m_editBarcode->setFocusPolicy(Qt::StrongFocus);
    m_editBarcode->setFocus();           // 카트 페이지 오면 스캐너 입력 받을 준비

    connect(m_editBarcode, SIGNAL(returnPressed()),this, SLOT(onBarcodeEntered()));

    qApp->installEventFilter(this);
    // 2) 서버 연동용 BarcodeScanner 생성
    m_scanner = new BarcodeScanner(this);

    connect(m_scanner, &BarcodeScanner::itemFetched,
            this, &PageCart::handleItemFetched);
    connect(m_scanner, &BarcodeScanner::fetchFailed,
            this, &PageCart::handleFetchFailed);

    // 3) 테이블 설정 + (테스트용) 더미 데이터
    ui->tableCart->setColumnCount(6);
    initDummyItems();                      // 필요 없으면 나중에 주석 처리
    updateTotal();

    connect(ui->btnGuideMode, SIGNAL(clicked()), this, SLOT(on_btnGuideMode_clicked()));
}

PageCart::~PageCart()
{
    delete ui;
}

// ----------------------------------------
// 더미 데이터 3개 생성
// ----------------------------------------
void PageCart::initDummyItems()
{
    QStringList names   = {"사과", "바나나", "우유"};
    QVector<int> prices = {3000, 1500, 2500};

    ui->tableCart->setRowCount(names.size());
    m_unitPrice = prices;

    for (int row = 0; row < names.size(); ++row) {

        // 상품명
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(names[row]));
        // 개수
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("0"));
        // 금액
        ui->tableCart->setItem(row, 4, new QTableWidgetItem("0"));

        // + 버튼 (col 2)
        QPushButton *btnPlus = new QPushButton("+", this);
        ui->tableCart->setCellWidget(row, 2, btnPlus);

        connect(btnPlus, SIGNAL(clicked()), this, SLOT(onPlusClicked()));

        // - 버튼 (col 3)
        QPushButton *btnMinus = new QPushButton("-", this);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));

        // 삭제 버튼 (col 5)
        QPushButton *btnDelete = new QPushButton("삭제", this);
        ui->tableCart->setCellWidget(row, 5, btnDelete);
        connect(btnDelete, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    }
}

// ----------------------------------------
// + 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 2) == btn) { // col 2 = +
            row = r;
            break;
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

// ----------------------------------------
// - 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 3) == btn) { // col 3 = -
            row = r;
            break;
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

// ----------------------------------------
// 삭제 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 5) == btn) { // col 5 = 삭제
            row = r;
            break;
        }
    }

    if (row < 0) return;

    // 단가 리스트에서도 삭제
    if (row < m_unitPrice.size())
        m_unitPrice.removeAt(row);

    // 테이블에서 행 삭제
    ui->tableCart->removeRow(row);

    updateTotal();
}

// ----------------------------------------
// 한 줄 금액 업데이트 → 개수 * 단가
// ----------------------------------------
void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;

    int qty = ui->tableCart->item(row, 1)->text().toInt();
    int amount = m_unitPrice[row] * qty;

    ui->tableCart->item(row, 4)->setText(QString::number(amount));
}

// ----------------------------------------
// 전체 총액 업데이트
// ----------------------------------------
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

    if (code.isEmpty())
        return;

    qDebug() << "[PageCart] barcode entered =" << code;

    // ✅ 여기서 서버로 요청 보내기
    m_scanner->fetchItemDetails(code);
}

bool PageCart::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        // F11: 전체 화면 토글 (MainWidget 전체를 기준으로)
        if (keyEvent->key() == Qt::Key_F11) {
            QWidget *top = this->window();   // 최상위 윈도우 (MainWidget)
            if (top->isFullScreen())
                top->showNormal();
            else
                top->showFullScreen();
            return true; // 이벤트 처리 완료
        }
        // Enter/Return: 지금까지 모은 바코드 문자열을 서버로 보냄
        else if (keyEvent->key() == Qt::Key_Return ||
                 keyEvent->key() == Qt::Key_Enter) {

            if (!m_barcodeData.isEmpty()) {
                qDebug() << "Final Barcode ID sent and processing:" << m_barcodeData;
                m_scanner->fetchItemDetails(m_barcodeData);
                m_barcodeData.clear();
            } else {
                qDebug() << "Enter pressed, but m_barcodeData is empty. Ignoring.";
            }
            return true;
        }
        // 일반 문자 키: 바코드 문자열에 누적
        else if (!keyEvent->text().isEmpty() &&
                 !(keyEvent->modifiers() & (Qt::ShiftModifier |
                                            Qt::ControlModifier |
                                            Qt::AltModifier))) {

            m_barcodeData.append(keyEvent->text());
            qDebug() << "Collecting barcode:" << m_barcodeData;
            return true;
        }
    }

    // 나머지 이벤트는 기본 처리로 넘김
    return QWidget::eventFilter(obj, event);
}

void PageCart::handleItemFetched(const Item &item)
{
    qDebug() << "[PageCart] handleItemFetched:" << item.name << item.price;

    // 1) 이미 테이블에 있는 상품인지 확인 (상품명 기준)
    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *nameItem = ui->tableCart->item(r, 0);
        if (nameItem && nameItem->text() == item.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        // 2-1) 없으면 새 행 추가
        int row = ui->tableCart->rowCount();
        ui->tableCart->insertRow(row);

        // 상품명 / 개수 / 금액
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(item.name));
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("1"));
        ui->tableCart->setItem(row, 4, new QTableWidgetItem(QString::number(item.price)));

        // 단가 리스트에도 추가 (기존 +/− 로직에서 사용)
        m_unitPrice.append(static_cast<int>(item.price));

        // + / - / 삭제 버튼 생성 (initDummyItems()에서 하던 것과 동일)
        QPushButton *btnPlus   = new QPushButton("+", this);
        QPushButton *btnMinus  = new QPushButton("-", this);
        QPushButton *btnDelete = new QPushButton("삭제", this);

        ui->tableCart->setCellWidget(row, 2, btnPlus);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        ui->tableCart->setCellWidget(row, 5, btnDelete);

        connect(btnPlus,  SIGNAL(clicked()), this, SLOT(onPlusClicked()));
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));
        connect(btnDelete,SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    }
    else {
        // 2-2) 이미 있는 상품이면 개수 +1
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

}

