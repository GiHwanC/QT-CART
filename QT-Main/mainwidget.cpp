#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "pagewelcome.h"
#include "pagecart.h"


MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // 페이지 위젯들 생성
    pPageWelcome = new PageWelcome(this);
    pPageCart = new PageCart(this);
    pPageGuide =  new PageGuide(this);

    // stackedWidget register
    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);
    ui->pstackedWidget->addWidget(pPageGuide);

    // first UI -> PageWelcome
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(pPageWelcome, SIGNAL(startClicked()), this, SLOT(on_pPBStartClicked()));
    connect(pPageGuide, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));
    connect(pPageCart, SIGNAL(guideModeClicked()), this, SLOT(slotShowGuidePage()));
    connect(pPageCart,SIGNAL(goWelcome()),this,SLOT(slotShowWelcomePage()));
}

MainWidget::~MainWidget()
{
    delete ui;
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

