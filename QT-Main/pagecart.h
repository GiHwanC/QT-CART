#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QString>
#include <QUdpSocket>
#include <QEvent>
#include <QPixmap>

#include "item.h"
#include "barcodescanner.h"

namespace Ui {
class PageCart;
}

/* =======================
 *  장바구니 아이템 구조체
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

    struct CartLine {
        QString name;
        int qty;
        int unitPrice;
    };

    QVector<CartLine> getCartLines() const;



    BarcodeScanner* scanner() const { return m_scanner; }

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;
public slots:
    void resetCart();
signals:
    void guideModeClicked();
    void goWelcome();
    void goPay();
    void payClicked();


private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();

    // BarcodeScanner 시그널(너 헤더에 있는 버전 유지)
    void handleItemFetched(const Item &item, double cartWeight);
    void handleFetchFailed(const QString &err);
    void on_pushButton_clicked();    // clear cart 버튼(pushButton)
    void on_btnCheckout_clicked();
    void on_btnGuide_clicked();


private:
    Ui::PageCart *ui;
bool m_weightFail = false;
    /* Cart Data */
    QVector<ItemInfo> m_items;
    QVector<int> m_unitPrice;
    const double m_tolerance = 30.0;

    /* Barcode */
    QLineEdit *m_editBarcode = nullptr;
    QString m_barcodeData;
    BarcodeScanner *m_scanner = nullptr;
    
    bool m_isStopped = false;
    
    /* Network */
    QUdpSocket *m_udpSocket = nullptr;

    /* Internal */
    void initDummyItems();
    void refreshCartFromServer(const QJsonObject& cart);

    void addRowForItem(const QString& name, int unitPrice, int qty);
    void updateRowAmount(int row);
    void updateTotal();

    /* Robot control */
    void sendRobotMode(int mode);

    /* Weight helpers */
    void addItemByScan(const Item &item);
    void updateExpectedWeightByScan(double itemWeight);
    bool checkWeightOrStop(double cartWeight);
    void requestCheckWeightBeforeRun();

};

#endif // PAGECART_H
