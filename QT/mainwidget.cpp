#include "mainwidget.h"
#include "ui_mainwidget.h"

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // 페이지 위젯들 생성
    pPageWelcome = new PageWelcome(this);
    pPageCart = new PageCart(this);

    // stackedWidget register
    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);

    // first UI -> PageWelcome
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(pPageWelcome, SIGNAL(startClicked()), this, SLOT(on_pPBStartClicked()));
}

MainWidget::~MainWidget()
{
    delete ui;
}

void MainWidget::on_pPBStartClicked(){
    ui->pstackedWidget->setCurrentWidget(pPageCart);
}
