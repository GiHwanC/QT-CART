#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QString>
#include <QUdpSocket>
#include <QEvent>
#include <QPixmap>

// 네트워크 및 JSON 처리
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>

#include "barcodescanner.h"

namespace Ui {
class PageCart;
}

/* 장바구니 아이템 구조체 */
struct ItemInfo {
    QString id;
    QString name;
    int price;
    double weight;
};

class PageCart : public QWidget
{
    Q_OBJECT

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();

    // 결제 페이지 데이터 전달용 구조체
    struct CartLine {
        QString name;
        int qty;
        int unitPrice;
    };

    QVector<CartLine> getCartLines() const;
    BarcodeScanner* scanner() const { return m_scanner; }

protected:
    // 키보드(바코드) 이벤트 감지
    bool eventFilter(QObject *obj, QEvent *event) override;

public slots:
    void resetCart(); // 카트 초기화

signals:
    void guideModeClicked();
    void goWelcome();
    void goPay(); // 무게 확인 통과 시 이동

private slots:
    // UI 조작 슬롯
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();

    // 스캐너 응답 처리
    void handleItemFetched(const Item &item);
    void handleFetchFailed(const QString &err);

    // 버튼 핸들러
    void on_pushButton_clicked();    // (구형) 카트 비우기
    void on_btnCheckout_clicked();   // 결제/출발
    void on_btnGuide_clicked();

private:
    Ui::PageCart *ui;

    /* Cart Data */
    QVector<ItemInfo> m_items;
    QVector<int> m_unitPrice;

    bool m_isCheckingOut;

    /* Barcode Internal */
    QLineEdit *m_editBarcode = nullptr;
    QString m_barcodeData;
    BarcodeScanner *m_scanner = nullptr;

    /* 내부 헬퍼 함수 */
    void addRowForItem(const QString& id, const QString& name, int unitPrice, int qty, double weight = 0.0);
    void updateRowAmount(int row);
    void updateTotal();
    int getRowFromButton(QWidget *btn);

    /* 핵심 로직 */
    void requestCheckWeightBeforeRun();
    void sendRobotMode(int mode);
};

#endif // PAGECART_H