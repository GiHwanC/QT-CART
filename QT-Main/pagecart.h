#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QString>
#include <QUdpSocket>
#include <QEvent>
#include <QPixmap>
#include <QTimer>

// ✅ 네트워크 통신 및 JSON 처리를 위한 헤더 추가
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonObject>
#include <QJsonDocument>

#include "item.h"
#include "barcodescanner.h"

namespace Ui {
class PageCart;
}

/* =======================
 * 장바구니 아이템 구조체
 * ======================= */
struct ItemInfo {
    int id;  
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

    // 결제 페이지 등으로 데이터를 넘길 때 사용하는 구조체
    struct CartLine {
        QString name;
        int qty;
        int unitPrice;
    };

    QVector<CartLine> getCartLines() const;
    BarcodeScanner* scanner() const { return m_scanner; }

protected:
    // 키보드(바코드 스캐너) 입력 감지용
    bool eventFilter(QObject *obj, QEvent *event) override;

public slots:
    void resetCart(); // 카트 초기화

signals:
    void guideModeClicked();
    void goWelcome();
    void goPay();       // 무게 체크 통과 시 결제 화면으로 이동
    void payClicked();  // (구형 시그널 유지)

private slots:
    // UI 버튼 이벤트 핸들러
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();

    // BarcodeScanner 응답 처리
    void handleItemFetched(const Item &item, double cartWeight);
    void handleFetchFailed(const QString &err);

    // 기타 UI 슬롯
    void on_pushButton_clicked();    // "카트 비우기" 버튼
    void on_btnCheckout_clicked();   // "결제/출발" 버튼
    void on_btnGuide_clicked();

private:
    Ui::PageCart *ui;
    
    /* Cart Data */
    QVector<ItemInfo> m_items;
    QVector<int> m_unitPrice;
    
    /* Barcode Internal */
    QLineEdit *m_editBarcode = nullptr;
    QString m_barcodeData;
    BarcodeScanner *m_scanner = nullptr;
    
    /* State */
    double m_expectedWeight = 0.0; 
    
    /* Network (UDP for Robot Control) */
    QUdpSocket *m_udpSocket = nullptr;

    /* 내부 헬퍼 함수들 */
    void initFixedItems(); // 초기 고정 상품 세팅
    void addRowForItem(const QString& name, int unitPrice, int qty);
    void updateRowAmount(int row);
    void updateTotal();
    void addItemByScan(const Item &item);

    /* 핵심 로직 함수들 */
    void requestCheckWeightBeforeRun(); // 출발 전 무게 체크 (HTTP)
    void sendRobotMode(int mode);       // 로봇 주행 명령 (UDP)
};

#endif // PAGECART_H