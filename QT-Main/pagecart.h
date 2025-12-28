#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QMap>
#include <QString>
#include "item.h"
#include "barcodescanner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QNetworkDatagram>

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
    void processPendingDatagrams();

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void guideModeClicked();
    void goWelcome();
    void goPay();

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
    QUdpSocket *m_udpSocket;

    // UWB 데이터
    float m_distL = 0.0;
    float m_distR = 0.0;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;

    // L, R 두 개의 값을 인자로 받음
    void controlDualRobot(float l, float r);
};

#endif // PAGECART_H
