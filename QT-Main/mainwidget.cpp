#include "mainwidget.h"
#include "ui_mainwidget.h"
#include <QDebug>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkDatagram>
#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QMessageBox>
#include <QUrl>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
    , m_isCheckingWeight(false)
{
    ui->setupUi(this);
    qApp->installEventFilter(this);

    m_udpSocket = new QUdpSocket(this);
    m_udpSocket->bind(QHostAddress::Any, QT_LISTEN_PORT, QUdpSocket::ShareAddress);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &MainWidget::onUdpReadyRead);

    m_networkManager = new QNetworkAccessManager(this);
    connect(m_networkManager, &QNetworkAccessManager::finished, this, &MainWidget::onCheckWeightResponse);

    pPageWelcome = new PageWelcome(this);
    pPageCart    = new PageCart(this);
    pPageCart->setApiConfig(API_SERVER_IP, API_SERVER_PORT);
    pPageGuide   = new PageGuide(this);
    pPagePay     = new PagePay(this);
    pPageCard    = new pagepay_card(this);
    pPageTotalPay= new PageTotalPay(this);

    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);
    ui->pstackedWidget->addWidget(pPageGuide);
    ui->pstackedWidget->addWidget(pPagePay);
    ui->pstackedWidget->addWidget(pPageCard);
    ui->pstackedWidget->addWidget(pPageTotalPay);
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(pPageWelcome, &PageWelcome::startRequested, this, &MainWidget::slotShowCartPage);

    connect(pPageGuide, &PageGuide::requestGoal, this, &MainWidget::onGoalRequestReceived); 
    connect(pPageGuide, &PageGuide::requestStop, this, &MainWidget::onStopRequestReceived); 
    connect(this, SIGNAL(robotArrivedSignal()), pPageGuide, SLOT(onRobotArrived()));        
    connect(pPageGuide, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));

    connect(pPageCart, SIGNAL(guideModeClicked()), this, SLOT(slotShowGuidePage()));
    connect(pPageCart, &PageCart::requestCheckout, this, &MainWidget::onCheckoutRequested);
    connect(pPageCart, SIGNAL(goWelcome()),        this, SLOT(slotShowWelcomePage()));

    connect(pPagePay, SIGNAL(creditCardClicked()), this, SLOT(slotShowPayCardPage()));
    connect(pPagePay, SIGNAL(backCartClicked()),   this, SLOT(slotShowCartPage()));

    connect(pPageCard, SIGNAL(goTotalPayClicked()), this, SLOT(slotShowTotalPayPage_2()));
    connect(pPageTotalPay, &PageTotalPay::backToStartClicked, this, &MainWidget::slotShowWelcomePage);

    connect(ui->pstackedWidget, &QStackedWidget::currentChanged, this, &MainWidget::onPageChanged);
    onPageChanged(ui->pstackedWidget->currentIndex());
}

MainWidget::~MainWidget()
{
    delete ui;
}

void MainWidget::sendUdpData(const QString &cmd)
{
    QByteArray data = cmd.toUtf8();
    m_udpSocket->writeDatagram(data, QHostAddress(ROS_SERVER_IP), ROS_SERVER_PORT);
    qDebug() << "[UDP Send]" << cmd << "to" << ROS_SERVER_IP;
}

void MainWidget::onCheckoutRequested(double expectedWeight)
{
    if (m_isCheckingWeight) {
        qDebug() << "Check skipped: Already verifying weight...";
        return;
    }
    m_isCheckingWeight = true; // Lock 설정
    
    QString urlStr = QString("http://%1:%2/cart/check_weight")
                        .arg(API_SERVER_IP)
                        .arg(API_SERVER_PORT);
                        
    QUrl url(urlStr);
    QNetworkRequest request(url);
    
    // GET 요청 전송 (응답은 onCheckWeightResponse 에서 처리)
    m_networkManager->get(request);
    
    qDebug() << "[API Request] Check Weight ->" << urlStr;
}

// ROS에서 온 UDP 수신
void MainWidget::onUdpReadyRead()
{
    while (m_udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        QString msg = QString::fromUtf8(datagram.data()).trimmed();
        
        qDebug() << "[UDP Recv]" << msg;

        if (msg == "STATUS:ARRIVED") {
            emit robotArrivedSignal(); 
        }
        else if (msg == "STATUS:WEIGHT_OK") { 
            // [성공] 무게가 일치함 -> 결제 페이지로 이동
            proceedToPayPage();
        }
        else if (msg == "ERROR:WEIGHT_MISMATCH") { // 데이터 파싱 (콜론 : 으로 나눔)
            QStringList parts = msg.split(":");
            
            double expected = 0.0;
            double real = 0.0;
            double diff = 0.0;

            // 데이터가 같이 왔는지 확인 (안 왔으면 0.0으로 표시됨)
            if (parts.size() >= 5) {
                expected = parts[2].toDouble();
                real     = parts[3].toDouble();
                diff     = parts[4].toDouble();
            }

            // 요청하신 팝업 코드 적용
            QString alertMsg = QString("상품 무게가 일치하지 않습니다.\n\n"
                                      "예상 무게: %1 g\n"
                                      "실제 무게: %2 g\n"
                                      "차이: %3 g\n\n"
                                      "카트의 물건을 확인해주세요.")
                                .arg(expected, 0, 'f', 1)
                                .arg(real, 0, 'f', 1)
                                .arg(diff, 0, 'f', 1);

            QMessageBox::warning(this, "출발 불가", alertMsg);
        }
    }
}

void MainWidget::proceedToPayPage()
{
    auto cartLines = pPageCart->getCartLines();
    QVector<PagePay::PayLine> payLines;
    payLines.reserve(cartLines.size());

    for (const auto &c : cartLines) {
        PagePay::PayLine p;
        p.name = c.name;
        p.qty = c.qty;
        p.unitPrice = c.unitPrice;
        payLines.push_back(p);
    }

    pPagePay->setPayItems(payLines);
    ui->pstackedWidget->setCurrentWidget(pPagePay);
}

void MainWidget::onGoalRequestReceived(double x, double y)
{
    QString cmd = QString("GOAL:%1,%2").arg(x).arg(y);
    sendUdpData(cmd);
}

void MainWidget::onStopRequestReceived()
{
    sendUdpData("STOP");
}

// 페이지 변경 시 모드 변경
void MainWidget::onPageChanged(int index)
{
    QWidget *currentWidget = ui->pstackedWidget->widget(index);
    int mode = 0;

    if (currentWidget == pPageWelcome) mode = 0;      // 정지
    else if (currentWidget == pPageCart) mode = 1;    // 따라가기
    else if (currentWidget == pPageGuide) mode = 2;   // 안내모드
    else if (currentWidget == pPagePay) mode = 0;     // 결제 중 정지 
    else mode = 0;

    sendRobotMode(mode);
}

void MainWidget::sendRobotMode(int mode)
{
    sendUdpData(QString("MODE:%1").arg(mode));
}

void MainWidget::on_pPBStartClicked() { ui->pstackedWidget->setCurrentWidget(pPageCart); }
void MainWidget::slotShowCartPage()   { ui->pstackedWidget->setCurrentWidget(pPageCart); }
void MainWidget::slotShowGuidePage()  { ui->pstackedWidget->setCurrentWidget(pPageGuide); }
void MainWidget::slotShowPayCardPage(){ ui->pstackedWidget->setCurrentWidget(pPageCard); }
void MainWidget::slotShowTotalPayPage_2() { ui->pstackedWidget->setCurrentWidget(pPageTotalPay); }

void MainWidget::slotShowWelcomePage()
{
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);
    pPageCart->resetCart();
}

void MainWidget::slotShowPayPage()
{
    auto cartLines = pPageCart->getCartLines();
    QVector<PagePay::PayLine> payLines;
    payLines.reserve(cartLines.size());

    for (const auto &c : cartLines) {
        PagePay::PayLine p;
        p.name = c.name;
        p.qty = c.qty;
        p.unitPrice = c.unitPrice;
        payLines.push_back(p);
    }

    pPagePay->setPayItems(payLines);
    ui->pstackedWidget->setCurrentWidget(pPagePay);
}

bool MainWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        auto *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_F12) {
            qApp->quit();
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

void MainWidget::onCheckWeightResponse(QNetworkReply *reply)
{
    // 요청한 URL이 check_weight 인지 확인 (다른 API 요청과 섞이지 않게)
    if (!reply->request().url().toString().contains("/cart/check_weight")) {
        return; 
    }

    if (reply->error()) {
        qDebug() << "API Error:" << reply->errorString();
        QMessageBox::critical(this, "통신 오류", "서버와 통신할 수 없습니다.\n" + reply->errorString());
        reply->deleteLater();
        m_isCheckingWeight = false;
        return;
    }

    QByteArray data = reply->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    QJsonObject obj = doc.object();

    // Python 서버 응답 필드: expected_weight, real_weight, diff, movable
    bool movable = obj["movable"].toBool();
    double expected = obj["expected_weight"].toDouble();
    double real = obj["real_weight"].toDouble();
    double diff = obj["diff"].toDouble();

    qDebug() << "[API Response] Movable:" << movable << "Diff:" << diff;

    if (movable) {
        // [성공] 오차 범위 내 -> 결제 페이지로 이동
        proceedToPayPage();
    } else {
        // [실패] 오차 범위 초과 -> 팝업 띄우기
        QString msg = QString("상품 무게가 일치하지 않습니다.\n\n"
                              "예상 무게: %1 g\n"
                              "실제 무게: %2 g\n"
                              "차이: %3 g\n\n"
                              "카트의 물건을 확인해주세요.")
                        .arg(expected, 0, 'f', 1)
                        .arg(real, 0, 'f', 1)
                        .arg(diff, 0, 'f', 1);

        QMessageBox::warning(this, "출발 불가", msg);
    }

    reply->deleteLater();
    
    m_isCheckingWeight = false;

}