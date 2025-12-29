#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QMap>
#include <QString>
#include "item.h"
#include "barcodescanner.h"

struct ItemInfo {
    QString name;
    int price;
    double weight;
};

namespace Ui {
class PageCart;
}

class PageCart : public QWidget
{
    Q_OBJECT

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();
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
    void handleItemFetched(const Item &item);
    void handleFetchFailed(const QString &err);
    void on_btnGuideMode_clicked();
    void on_pushButton_clicked();
    void on_btnPay_clicked();

private:
    Ui::PageCart *ui;
    QPixmap m_cartPixmap;

    void initDummyItems();
    void updateRowAmount(int row);
    void updateTotal();
    void createRow(int row, const QString &name, int price, int qty);
    void updateRowAmount(int row, int qty);

    QVector<int> m_unitPrice;
    QLineEdit *m_editBarcode;
    BarcodeScanner *m_scanner;
    QString m_barcodeData;
};

#endif // PAGECART_H
