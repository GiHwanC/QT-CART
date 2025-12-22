#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QMap>
#include <QString>
#include <QtNetwork/QUdpSocket>
#include "item.h"
#include "barcodescanner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

struct ItemInfo {
    QString name;
    int price;
    double weight;   // 나중에 로드셀 검증용
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

public:

    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void guideModeClicked();

private slots:
    // 기존 UI 슬롯
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();
    void handleItemFetched(const Item &item);
    void handleFetchFailed(const QString &err);
    void on_btnGuideMode_clicked();

    void readUwbUdp();

private:
    Ui::PageCart *ui;
    QPixmap m_cartPixmap;
    void initDummyItems();
    void updateRowAmount(int row);
    void updateTotal();
    void handleBarcode(const QString &code);   // ⬅ 바코드 처리용
    void createRow(int row, const QString &name, int price, int qty);
    void updateRowAmount(int row, int qty);
    QVector<int> m_unitPrice;
    QLineEdit *m_editBarcode;
    BarcodeScanner *m_scanner;
    QString m_barcodeData;

    // [네트워크]
    QUdpSocket *m_udpSocket;

    // [UWB 데이터 저장용 변수 추가]
    float m_distL = 0.0;
    float m_distR = 0.0;

    // [ROS 2 제어 관련]
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;

    // [수정된 제어 로직] L, R 두 개의 값을 인자로 받음
    void controlDualRobot(float l, float r);
};

#endif // PAGECART_H
