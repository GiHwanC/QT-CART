#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);

    QPixmap map(":/house.pgm");

    if (map.isNull()) {
        qDebug() << "map load failed";
    } else {
        ui->labelMap->setPixmap(map);
        ui->labelMap->setScaledContents(true);
    }

    connect(ui->btnBackToCart, SIGNAL(clicked()), this, SLOT(on_btnBackToCart_clicked()));
}

PageGuide::~PageGuide()
{
    delete ui;
}

void PageGuide::on_btnBackToCart_clicked()
{
    emit backToCartClicked();
}


void PageGuide::on_foodIcon_clicked()
{

}


void PageGuide::on_groceryIcon_clicked()
{

}

