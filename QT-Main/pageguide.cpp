#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>
#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QResizeEvent>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);

    // 버튼 연결
    connect(ui->btnFindItem,   &QPushButton::clicked, this, &PageGuide::onFindItemClicked);
    connect(ui->btnStopGuide,  &QPushButton::clicked, this, &PageGuide::onStopGuideClicked);
    connect(ui->btnBackToCart, &QPushButton::clicked, this, &PageGuide::onBackToCartClicked);

    // ✅ 이미지(임시) - 너가 나중에 바꿔서 쓰면 됨
    // qrc에 tree 이미지 넣고 경로만 바꿔줘
    m_treePixmap = QPixmap(":/item/cart_handcream.jpg");


    // 버튼/배경 스타일(원하면 수정)
    ui->btnFindItem->setStyleSheet(
        "QPushButton{background:#2563EB;color:white;border:none;border-radius:10px;padding:10px 18px;}"
        "QPushButton:hover{background:#1D4ED8;}"
        );
    ui->btnStopGuide->setStyleSheet(
        "QPushButton{background:#EF4444;color:white;border:none;border-radius:10px;padding:10px 18px;}"
        "QPushButton:hover{background:#DC2626;}"
        );
    ui->btnBackToCart->setStyleSheet(
        "QPushButton{background:#16A34A;color:white;border:none;border-radius:12px;padding:12px 20px;font-size:16px;}"
        "QPushButton:hover{background:#15803D;}"
        );

    // 이미지 라벨도 살짝 카드 느낌
    ui->lblmap->setStyleSheet("border-radius:14px; background:#ffffff;");
}

PageGuide::~PageGuide()
{
    delete ui;
}


void PageGuide::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);
    applyTreePixmap();
}

void PageGuide::applyTreePixmap()
{
    if (m_treePixmap.isNull() || !ui->lblmap) return;

    // 라벨 크기에 맞춰 비율 유지로 스케일
    QSize target = ui->lblmap->size();
    QPixmap scaled = m_treePixmap.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui->lblmap->setPixmap(scaled);
}

void PageGuide::onFindItemClicked()
{
    QMessageBox::information(this, "물건찾기", "물건 찾기 기능이 실행됩니다.");
}


// void PageGuide::on_foodIcon_clicked()
// {
//     qDebug() << "Food Icon Clicked";
//     emit requestGoal(-0.8, 0.0);
// }

// void PageGuide::on_groceryIcon_clicked()
// {
//     qDebug() << "Grocery Icon Clicked";
//     emit requestGoal(2.0, 0.0);
// }

void PageGuide::onStopGuideClicked()
{
    QMessageBox::information(this, "안내중지", "안내가 중지됩니다.");
}

void PageGuide::onBackToCartClicked()
{
    emit backToCartClicked(); // ✅ MainWidget에서 cart로 전환

}
