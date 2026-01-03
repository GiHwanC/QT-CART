#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QtNetwork/QUdpSocket>

#include "pagewelcome.h"
#include "pagecart.h"
#include "pageguide.h"
#include "pagepay.h"
#include "pagepay_card.h"
#include "pagetotalpay.h"

// 우분투 중계 서버
#define ROS_SERVER_IP   "192.168.123.42"
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
    QTimer *m_weightCheckTimer;

    // 모드 전송 함수 (0:정지, 1:따라가기, 2:안내)
    void sendRobotMode(int mode);
    void sendGoalCommand(double x, double y);

    pagepay_card *pPageCard;
    PageTotalPay *pPageTotalPay;

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;
private slots:

    void onGuideCancel();
    void on_pPBStartClicked();
    void slotShowCartPage();
    void slotShowGuidePage();
    void slotShowWelcomePage();
    void onGoalRequested(double x, double y);
    void onPageChanged(int index);
    void slotShowPayCardPage();
    void slotShowPayPage();
    void slotShowTotalPayPage_2();
};
#endif // MAINWIDGET_H
