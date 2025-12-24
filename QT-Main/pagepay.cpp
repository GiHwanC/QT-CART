#include "pagepay.h"
#include "ui_pagepay.h"

PagePay::PagePay(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PagePay)
{
    ui->setupUi(this);
}

PagePay::~PagePay()
{
    delete ui;
}
