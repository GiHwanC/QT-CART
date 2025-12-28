#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include "pagewelcome.h"
#include "pagecart.h"
#include "pageguide.h"
#include "pagepay.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private:
    Ui::MainWidget *ui;
    PageWelcome *pPageWelcome;
    PageCart *pPageCart;
    PageGuide *pPageGuide;
    PagePay *pPagePay;

    // rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

private slots:
    void on_pPBStartClicked();
    void slotShowCartPage();
    void slotShowGuidePage();
    void slotShowWelcomePage();
    void slotShowPayPage();
};
#endif // MAINWIDGET_H
