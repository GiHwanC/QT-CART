#include "pagepay_card.h"
#include "ui_pagepay_card.h"

pagepay_card::pagepay_card(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::pagepay_card)
{
    ui->setupUi(this);
}

pagepay_card::~pagepay_card()
{
    delete ui;
}
