#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QString>

#include "barcodescanner.h" 

namespace Ui { class PageCart; }

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

    struct CartLine {
        QString name;
        int qty;
        int unitPrice;
    };

    QVector<CartLine> getCartLines() const;
    
    // MainWidget에서 API 설정 받기
    void setApiConfig(const QString &ip, int port);

    double getTotalWeight() const;

public slots:
    void resetCart(); 

signals:
    void guideModeClicked();
    void goWelcome();
    void requestCheckout(double totalWeight);

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();

    void handleItemFetched(const Item &item);
    void handleFetchFailed(const QString &err);

    void on_pushButton_clicked();    // 카트 비우기
    void on_btnCheckout_clicked();   // 결제 버튼
    void on_btnGuide_clicked();      // 안내 버튼

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    Ui::PageCart *ui;
    
    QVector<ItemInfo> m_items;
    QVector<int> m_unitPrice;
    
    QLineEdit *m_editBarcode = nullptr;
    QString m_barcodeData;
    BarcodeScanner *m_scanner = nullptr; 
    
    void addRowForItem(const QString& id, const QString& name, int unitPrice, int qty, double weight);
    void updateRowAmount(int row);
    void updateTotal();
    int getRowFromButton(QWidget *btn);
};




#endif // PAGECART_H
