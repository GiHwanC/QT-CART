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
    qDebug() << "right Clicked";
    emit requestGoal(0.0, -6.83);
}

void PageGuide::on_groceryIcon_clicked()
{
    qDebug() << "left Clicked";
    emit requestGoal(0.0, -1.0);
}
