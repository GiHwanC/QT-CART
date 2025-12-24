#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QMap>
#include <QString>
#include <QTimer>
#include "uwbdriver.h"
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

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void guideModeClicked();

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();
    void onBarcodeEntered();
    void handleItemFetched(const Item &item);
    void handleFetchFailed(const QString &err);
    void on_btnGuideMode_clicked();

    void onUwbTimerTimeout();

private:
    Ui::PageCart *ui;

    void initDummyItems();
    void updateRowAmount(int row);
    void updateTotal();

    QVector<int> m_unitPrice;
    QLineEdit *m_editBarcode;
    BarcodeScanner *m_scanner;
    QString m_barcodeData;
    UwbDriver *m_uwbDriver;
    QTimer *m_uwbTimer;

    // UWB 데이터
    float m_distL = 0.0;
    float m_distR = 0.0;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;

    // L, R 두 개의 값을 인자로 받음
    void controlDualRobot(float l, float r);
};

#endif // PAGECART_H
