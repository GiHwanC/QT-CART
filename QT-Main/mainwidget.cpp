#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "pagewelcome.h"
#include "pagecart.h"
#include "pagepay.h"
#include <QDebug>
#include <QtNetwork/QHostAddress>
#include "pagepay_card.h"
#include "pagetotalpay.h"
#include <QTimer>
#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);
    qApp->installEventFilter(this);
    m_udpSocket = new QUdpSocket(this);

    pPageWelcome = new PageWelcome(this);
    pPageCart    = new PageCart(this);
    pPageGuide   = new PageGuide(this);
    pPagePay     = new PagePay(this);
    pPageCard    = new pagepay_card(this);
    pPageTotalPay = new PageTotalPay(this);
    // stackedWidget register

    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);
    ui->pstackedWidget->addWidget(pPageGuide);
    ui->pstackedWidget->addWidget(pPagePay);
    ui->pstackedWidget->addWidget(pPageCard);
    ui->pstackedWidget->addWidget(pPageTotalPay);
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(pPageWelcome, &PageWelcome::startRequested,
            this, &MainWidget::slotShowCartPage);

    connect(pPageGuide, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));
    connect(ui->pstackedWidget, &QStackedWidget::currentChanged, this, &MainWidget::onPageChanged);
    connect(pPageGuide, &PageGuide::requestGoal, this, &MainWidget::onGoalRequested);

    connect(pPageCart,  SIGNAL(guideModeClicked()),  this, SLOT(slotShowGuidePage()));
    connect(pPageCart, SIGNAL(goPay()), this, SLOT(slotShowPayPage()));
    connect(pPageCart,  SIGNAL(goWelcome()),         this, SLOT(slotShowWelcomePage()));
    connect(pPageTotalPay, &PageTotalPay::backToStartClicked,
            this, &MainWidget::slotShowWelcomePage);

    connect(pPagePay,   SIGNAL(creditCardClicked()), this, SLOT(slotShowPayCardPage()));
    connect(pPagePay, SIGNAL(backCartClicked()), this, SLOT(slotShowCartPage()));
    connect(pPagePay,   SIGNAL(backCartClicked()),   this, SLOT(slotShowCartPage()));

    connect(pPageCard, SIGNAL(goTotalPayClicked()), this, SLOT(slotShowTotalPayPage_2()));
    connect(pPageTotalPay, &PageTotalPay::backToStartClicked,
            this, &MainWidget::slotShowWelcomePage);

    connect(pPageGuide, &PageGuide::requestCancelGoal,
            this, &MainWidget::onGuideCancel);
    onPageChanged(ui->pstackedWidget->currentIndex());
}

MainWidget::~MainWidget()
{
    delete ui;
}

void MainWidget::onPageChanged(int index)
{
    // 현재 화면
    QWidget *currentWidget = ui->pstackedWidget->widget(index);

    int mode = 0;

    if (currentWidget == pPageWelcome) mode = 0; // 정지
    else if (currentWidget == pPageCart) mode = 1; // 따라가기 모드
    else if (currentWidget == pPageGuide) mode = 2; // 안내 모드
    else if (currentWidget == pPagePay) mode = 0; // 결제 중 정지
    else {
        mode = 0;
        qDebug() << "Unknown-> Mode 0 (STOP)";
    }

    sendRobotMode(mode);
}

void MainWidget::sendRobotMode(int mode)
{
    QString cmd = QString("MODE:%1").arg(mode);
    QByteArray data = cmd.toUtf8();

    m_udpSocket->writeDatagram(
        data,
        QHostAddress(ROS_SERVER_IP),
        ROS_SERVER_PORT
    );
}

void MainWidget::onGoalRequested(double x, double y)
{
    sendGoalCommand(x, y);
}

void MainWidget::sendGoalCommand(double x, double y)
{
    // 형식: "GOAL:1.5,-0.5"
    QString cmd = QString("GOAL:%1,%2").arg(x).arg(y);
    QByteArray data = cmd.toUtf8();

    m_udpSocket->writeDatagram(data, QHostAddress(ROS_SERVER_IP), ROS_SERVER_PORT);
    qDebug() << "UDP Send:" << cmd;
}

void MainWidget::on_pPBStartClicked(){
    ui->pstackedWidget->setCurrentWidget(pPageCart);
}
void MainWidget::slotShowGuidePage()
{
    ui->pstackedWidget->setCurrentWidget(pPageGuide);
}
void MainWidget::slotShowCartPage()
{
    ui->pstackedWidget->setCurrentWidget(pPageCart);
}
void MainWidget::slotShowWelcomePage()
{
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);
    pPageCart->resetCart();
}
void MainWidget::slotShowPayPage()
{
// <<<<<<< HEAD
//     ui->pstackedWidget->setCurrentWidget(pPagePay);
// =======
    auto cartLines = pPageCart->getCartLines();

    // 2) PagePay 형식으로 변환해서 전달
    QVector<PagePay::PayLine> payLines;
    payLines.reserve(cartLines.size());

    for (const auto &c : cartLines) {
        PagePay::PayLine p;
        p.name = c.name;
        p.qty = c.qty;
        p.unitPrice = c.unitPrice;
        payLines.push_back(p);
    }

    // 3) Pay 페이지 테이블 갱신
    pPagePay->setPayItems(payLines);

    // 4) 페이지 이동
    ui->pstackedWidget->setCurrentWidget(pPagePay);

}
void MainWidget::slotShowPayCardPage()
{
    ui->pstackedWidget->setCurrentWidget(pPageCard);
}


void MainWidget::slotShowTotalPayPage_2()
{
    ui->pstackedWidget->setCurrentWidget(pPageTotalPay);
}
bool MainWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        auto *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_F12) {
            qApp->quit();         // ✅ 프로그램 종료
            return true;          // 이벤트 먹기(다른 곳으로 안 넘어감)
        }
    }
    return QWidget::eventFilter(obj, event);
}


void MainWidget::onGuideCancel()
{
    // ✅ (가능하면) 네비게이션 goal cancel도 같이
    // 예: navClient->cancelGoal();

    // ✅ 핵심: 화면이 안 바뀌어도 즉시 정지 모드 전송
    sendRobotMode(0);
}
