#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>
#include <QDebug>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);

    QPixmap map(":/house.pgm");
    if (map.isNull()) {
        qDebug() << "[PageGuide] Map load failed. Check qrc file.";
    } else {
        ui->labelMap->setPixmap(map);
        ui->labelMap->setScaledContents(true);
    }
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
    qDebug() << "Food Icon Clicked";
    emit requestGoal(-0.8, 0.0);
}

void PageGuide::on_groceryIcon_clicked()
{
    qDebug() << "Grocery Icon Clicked";
    emit requestGoal(2.0, 0.0);
}
