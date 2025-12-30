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

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void guideModeClicked();
    void goWelcome();
    void goPay();

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();

    void handleItemFetched(const Item &item, double cartWeight);
    void handleFetchFailed(const QString &err);

    void on_btnGuideMode_clicked();
    void on_pushButton_clicked();
    void on_btnPay_clicked();

private:
    /* UI */
    Ui::PageCart *ui;
    QPixmap m_cartPixmap;
    void addItemByScan(const Item &item);
    void updateExpectedWeightByScan(double itemWeight);
    bool checkWeightOrStop(double cartWeight);
    /* Cart Data */
    QVector<ItemInfo> m_items;     // ✅ 반드시 필요
    QVector<int> m_unitPrice;

    double m_expectedWeight = 0.0;
    const double m_tolerance = 30.0;

    /* Barcode */
    QLineEdit *m_editBarcode;
    QString m_barcodeData;
    BarcodeScanner *m_scanner;

    /* Network */
    QUdpSocket *m_udpSocket;

    /* Internal */
    void initDummyItems();
    void updateRowAmount(int row);
    void updateTotal();

    /* Robot control */
    void sendRobotMode(int mode);  // ✅ 핵심
};

#endif // PAGECART_H
