#include "pagepay.h"
#include "ui_pagepay.h"

PagePay::PagePay(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PagePay)
{
    ui->setupUi(this);

    connect(ui->btnBackToCart, SIGNAL(clicked()), this, SLOT(on_btnBackToCart_clicked()));
}

PagePay::~PagePay()
{
    delete ui;
}
void PagePay::on_btnBackToCart_clicked()
{
    emit backToCartClicked();
}
