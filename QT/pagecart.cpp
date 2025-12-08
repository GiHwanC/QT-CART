#include "pagecart.h"
#include "ui_pagecart.h"

PageCart::PageCart(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageCart)
{
    ui->setupUi(this);
}

PageCart::~PageCart()
{
    delete ui;
}
