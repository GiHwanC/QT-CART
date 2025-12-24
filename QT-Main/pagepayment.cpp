#include "pagepayment.h"
#include "ui_pagepayment.h"

pagepayment::pagepayment(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::pagepayment)
{
    ui->setupUi(this);
}

pagepayment::~pagepayment()
{
    delete ui;
}
