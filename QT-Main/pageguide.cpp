#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>
#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QResizeEvent>
#include <QDialog>
#include <QLabel>
#include <QTimer>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);
    QPixmap pm(":/etc/guide_map.png");
    ui->lblmap->setPixmap(pm);
    ui->lblmap->setScaledContents(true);   // 라벨 크기에 맞게 꽉 채움
    ui->lblmap->setStyleSheet("QLabel { border: none; }");

    // 버튼 연결

    connect(ui->btnBackToCart, &QPushButton::clicked, this, &PageGuide::onBackToCartClicked);


    // ✅ 지도 위 버튼들 연결
    connect(ui->btnpuzzle, &QPushButton::clicked, this, &PageGuide::onPuzzleClicked);
    connect(ui->btncream,  &QPushButton::clicked, this, &PageGuide::onCreamClicked);
    connect(ui->btnsnack,  &QPushButton::clicked, this, &PageGuide::onSnackClicked);
    connect(ui->btnpay,    &QPushButton::clicked, this, &PageGuide::onPayClicked);
    connect(ui->btnphone,    &QPushButton::clicked, this, &PageGuide:: onbtnphoneClicked);




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



void PageGuide::onBackToCartClicked()
{
    emit backToCartClicked(); // ✅ MainWidget에서 cart로 전환

}

void PageGuide::onPuzzleClicked()
{
    showMovePopup("01코너 퍼즐");

    qDebug() << "Region2 Icon Clicked";
    emit requestGoal(0.0, 1.0);
}


void PageGuide::onSnackClicked()
{
    showMovePopup("02코너 과자");

    qDebug() << "Region2 Icon Clicked";
    emit requestGoal(0.0, 1.0);
}


void PageGuide::onbtnphoneClicked()
{
    showMovePopup("04코너 핸드폰");

    qDebug() << "Region2 Icon Clicked";
    emit requestGoal(0.0, 1.0);
}


void PageGuide::onCreamClicked()
{
    showMovePopup("03코너 핸드크림");

    qDebug() << "Region2 Icon Clicked";
    emit requestGoal(0.0, 1.0);
}


void PageGuide::onPayClicked()
{
    showMovePopup("결제 구역");

    qDebug() << "Region2 Icon Clicked";
    emit requestGoal(0.0, 1.0);
}
void PageGuide::showMovePopup(const QString &zoneText)
{
    QWidget *overlayParent = this->window();
    if (!overlayParent) overlayParent = this;

    // ✅ 프레임리스 모달 다이얼로그(전체 화면 덮기)
    QDialog *dlg = new QDialog(overlayParent);
    dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    dlg->setModal(true);
    dlg->setAttribute(Qt::WA_TranslucentBackground);

    // 부모 창 크기 그대로 덮기
    dlg->setGeometry(overlayParent->rect());

    // ✅ 반투명 배경
    QFrame *bg = new QFrame(dlg);
    bg->setGeometry(dlg->rect());
    bg->setStyleSheet("QFrame{ background-color: rgba(0,0,0,140); }");

    QVBoxLayout *root = new QVBoxLayout(bg);
    root->setContentsMargins(0,0,0,0);

    // ✅ 가운데 카드
    QFrame *card = new QFrame(bg);
    card->setFixedSize(520, 320);
    card->setStyleSheet(
        "QFrame{ background:white; border-radius:16px; }"
        );

    root->addStretch();
    root->addWidget(card, 0, Qt::AlignHCenter);
    root->addStretch();

    QVBoxLayout *lay = new QVBoxLayout(card);
    lay->setContentsMargins(28, 24, 28, 24);
    lay->setSpacing(14);

    QLabel *title = new QLabel(QString("%1로 이동중입니다").arg(zoneText), card);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size:24px; font-weight:900; color:#111827;");

    QLabel *desc = new QLabel("화면을 종료하거나 터치하지 마십시오", card);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size:16px; font-weight:700; color:#6B7280;");

    QPushButton *ok = new QPushButton("확인", card);
    ok->setFixedHeight(48);
    ok->setStyleSheet(
        "QPushButton{ background:#2563EB; color:white; border:none; border-radius:12px; "
        "font-size:18px; font-weight:900; padding:8px 18px; }"
        "QPushButton:hover{ background:#1D4ED8; }"
        "QPushButton:pressed{ background:#1E40AF; }"
        );

    lay->addStretch();
    lay->addWidget(title);
    lay->addWidget(desc);
    lay->addSpacing(8);
    lay->addWidget(ok, 0, Qt::AlignHCenter);
    lay->addStretch();

    connect(ok, &QPushButton::clicked, dlg, &QDialog::accept);

    // ✅ 실행
    dlg->exec();
    dlg->deleteLater();
}
