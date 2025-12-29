#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "pagewelcome.h"
#include "pagecart.h"
#include "pagepay.h"

#include <QDebug>
#include <QtNetwork/QHostAddress>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    m_udpSocket = new QUdpSocket(this);

    pPageWelcome = new PageWelcome(this);
    pPageCart = new PageCart(this);
    pPageGuide =  new PageGuide(this);
    pPagePay = new PagePay(this);

    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);
    ui->pstackedWidget->addWidget(pPageGuide);
    ui->pstackedWidget->addWidget(pPagePay);

    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(ui->pstackedWidget, &QStackedWidget::currentChanged, this, &MainWidget::onPageChanged);
    connect(pPageGuide, &PageGuide::requestGoal, this, &MainWidget::onGoalRequested);

    connect(pPageWelcome, SIGNAL(startClicked()), this, SLOT(on_pPBStartClicked()));
    connect(pPageGuide, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));
    connect(pPageCart, SIGNAL(guideModeClicked()), this, SLOT(slotShowGuidePage()));
    connect(pPageCart,SIGNAL(goWelcome()),this, SLOT(slotShowWelcomePage()));
    connect(pPageCart, SIGNAL(goPay()), this, SLOT(slotShowPayPage()));
    connect(pPagePay, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));
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
    // 형식: "MODE:1"
    QString cmd = QString("MODE:%1").arg(mode);
    QByteArray data = cmd.toUtf8();

    m_udpSocket->writeDatagram(data, QHostAddress(ROS_SERVER_IP), ROS_SERVER_PORT);
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
}
void MainWidget::slotShowPayPage()
{
    ui->pstackedWidget->setCurrentWidget(pPagePay);
}
