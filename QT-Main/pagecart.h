#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QString>
#include <QUdpSocket>
#include <QEvent>
#include <optional>

#include "item.h"
#include "barcodescanner.h"
#include "carttypes.h"

namespace Ui {
class PageCart;
}

struct PendingRemove {
    int itemId;
    int remaining;
};

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

signals:
    void guideModeClicked();
    void goWelcome();
    void goPay();

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();
    void on_pushButton_clicked();
    void on_btnCheckout_clicked();
    void on_btnGuide_clicked();
    void onCartResult(const CartServerResult& r);

public slots:
    void resetCart();

private:
    Ui::PageCart *ui;

    /* Cart state */
    QVector<ItemInfo> m_items;
    QVector<int> m_unitPrice;
    std::optional<PendingRemove> m_pendingRemove;

    bool m_checkingWeight = false;
    /* Weight */
    const double m_tolerance = 30.0;
    double m_expectedWeight = 0.0;
    bool m_isStopped = false;

    /* Barcode */
    QLineEdit *m_editBarcode = nullptr;
    QString m_barcodeData;
    BarcodeScanner *m_scanner = nullptr;

    /* UI helpers */
    void addRowForItem(const QString& name, int unitPrice, int qty);
    void updateRowAmount(int row);
    void updateTotal();
    void applyRemoveOneToUi(int itemId);
    void addItemByScan(const Item &item);

    /* Robot */
    void sendRobotMode(int mode);
};

#endif // PAGECART_H
