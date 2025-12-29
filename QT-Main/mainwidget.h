#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QtNetwork/QUdpSocket>

#include "pagewelcome.h"
#include "pagecart.h"
#include "pageguide.h"
#include "pagepay.h"

// 우분투 중계 서버
#define ROS_SERVER_IP   "10.10.14.92"
#define ROS_SERVER_PORT 55555

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
    QUdpSocket *m_udpSocket;

    // 모드 전송 함수 (0:정지, 1:따라가기, 2:안내)
    void sendRobotMode(int mode);
    void sendGoalCommand(double x, double y);

private slots:
    void on_pPBStartClicked();
    void slotShowCartPage();
    void slotShowGuidePage();
    void slotShowWelcomePage();
    void slotShowPayPage();

    void onPageChanged(int index);
    void onGoalRequested(double x, double y);
};
#endif // MAINWIDGET_H
