#include "pagewelcome.h"
#include "ui_pagewelcome.h"

PageWelcome::PageWelcome(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageWelcome)
{
    ui->setupUi(this);

    connect(ui->pPBStart, &QPushButton::clicked, this, &PageWelcome::startClicked);
}

PageWelcome::~PageWelcome(){
    delete ui;
}
