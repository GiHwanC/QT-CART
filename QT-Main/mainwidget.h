#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QtNetwork/QUdpSocket>

#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>
#include <QJsonDocument>
#include <QJsonObject>

#define ROS_SERVER_IP   "192.168.123.42"
#define ROS_SERVER_PORT 55555

#define API_SERVER_IP   "192.168.123.43"
#define API_SERVER_PORT 8000

#define QT_LISTEN_PORT  55555

#include "pagewelcome.h"
#include "pagecart.h"
#include "pageguide.h"
#include "pagepay.h"
#include "pagepay_card.h"
#include "pagetotalpay.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWidget; }
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

signals:
    void robotArrivedSignal(); 

private:
    Ui::MainWidget *ui;
    QUdpSocket *m_udpSocket; 

    QNetworkAccessManager *m_networkManager;

    PageWelcome *pPageWelcome;
    PageCart *pPageCart;
    PageGuide *pPageGuide;
    PagePay *pPagePay;
    pagepay_card *pPageCard;
    PageTotalPay *pPageTotalPay; 

    bool m_isCheckingWeight;

    void sendUdpData(const QString &cmd);   
    void sendRobotMode(int mode); // 모드 전송 (0:정지, 1:따라가기, 2:안내)

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onUdpReadyRead(); // UDP 수신부 (무게 검증 결과 처리)
    
    void onCheckWeightResponse(QNetworkReply *reply);

    void on_pPBStartClicked(); 
    void slotShowPayPage();
    
    void slotShowCartPage();
    void slotShowGuidePage();
    void slotShowPayCardPage();
    void slotShowTotalPayPage_2();
    void slotShowWelcomePage();
    
    void onCheckoutRequested(double expectedWeight);
    
    void proceedToPayPage();

    void onGoalRequestReceived(double x, double y);
    void onStopRequestReceived();
    void onPageChanged(int index);
};

#endif // MAINWIDGET_H