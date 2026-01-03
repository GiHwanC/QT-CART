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
    emit requestGoal(-0.1, -2.4);

    showMovePopupAsync("01코너 퍼즐");

    qDebug() << "Region2 Icon Clicked";
}

void PageGuide::onSnackClicked()
{
    emit requestGoal(3.0, 0.55);

    showMovePopupAsync("02코너 과자");


    qDebug() << "Region2 Icon Clicked";
}

void PageGuide::onCreamClicked()
{
    emit requestGoal(0.021, -5.0);

    showMovePopupAsync("03코너 핸드크림");

    qDebug() << "Region2 Icon Clicked";
}

void PageGuide::onbtnphoneClicked()
{
    emit requestGoal(-0.1, -5.0);

    showMovePopupAsync("04코너 핸드폰");


    qDebug() << "Region2 Icon Clicked";
}

void PageGuide::onPayClicked()
{
    emit requestGoal(-0.07, -6.7);

    showMovePopupAsync("결제 구역");


    qDebug() << "Region2 Icon Clicked";
}
#include <QGraphicsDropShadowEffect>

void PageGuide::showMovePopup(const QString &zoneText)
{
    QWidget *parentW = this->window();
    if (!parentW) parentW = this;

    // 이미 팝업 떠있으면 닫고 새로 띄움
    if (m_moveDlg) {
        m_moveDlg->close();
        m_moveDlg = nullptr;
    }

    QDialog *dlg = new QDialog(parentW);
    m_moveDlg = dlg;

    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    dlg->setModal(true);
    dlg->setStyleSheet("QDialog { background: transparent; }");
    dlg->setFixedSize(520, 260);

    // 중앙 배치
    const QPoint parentTopLeft = parentW->mapToGlobal(QPoint(0, 0));
    const int x = parentTopLeft.x() + (parentW->width()  - dlg->width())  / 2;
    const int y = parentTopLeft.y() + (parentW->height() - dlg->height()) / 2;
    dlg->move(x, y);

    // 카드
    QFrame *card = new QFrame(dlg);
    card->setObjectName("popupCard");
    card->setStyleSheet("#popupCard { background: rgb(246,245,244); border-radius: 16px; }");

    auto *shadow = new QGraphicsDropShadowEffect(card);
    shadow->setBlurRadius(30);
    shadow->setOffset(0, 8);
    shadow->setColor(QColor(0, 0, 0, 80));
    card->setGraphicsEffect(shadow);

    QVBoxLayout *root = new QVBoxLayout(dlg);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(card);

    QVBoxLayout *lay = new QVBoxLayout(card);
    lay->setContentsMargins(28, 24, 28, 24);
    lay->setSpacing(14);

    QLabel *title = new QLabel(QString("%1로 이동 중입니다").arg(zoneText), card);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size:24px; font-weight:900; color:#111827;");

    QLabel *desc = new QLabel("확인을 누르면 이동을 중단합니다", card);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size:16px; font-weight:700; color:#6B7280;");

    QPushButton *ok = new QPushButton("확인(이동 중단)", card);
    ok->setFixedHeight(48);
    ok->setMinimumWidth(260);
    ok->setStyleSheet(
        "QPushButton{ background:#EF4444; color:white; border:none; border-radius:12px; "
        "font-size:18px; font-weight:900; padding:8px 18px; }"
        "QPushButton:hover{ background:#DC2626; }"
        "QPushButton:pressed{ background:#B91C1C; }"
        );

    lay->addStretch();
    lay->addWidget(title);
    lay->addWidget(desc);
    lay->addSpacing(8);
    lay->addWidget(ok, 0, Qt::AlignHCenter);
    lay->addStretch();

    // ✅ 확인 누르면: 이동 취소 + 팝업 닫기
    connect(ok, &QPushButton::clicked, this, [this, dlg]() {
        emit requestCancelGoal();
        dlg->close();
    });

    // ✅ 팝업이 어떤 이유로든 닫히면(뒤로가기/강제 close 포함) 이동 취소
    connect(dlg, &QDialog::finished, this, [this](int) {
        emit requestCancelGoal();
        m_moveDlg = nullptr;
    });

    dlg->open(); // ✅ 모달이지만 "비동기" (코드 안 멈춤)
}
void PageGuide::showMovePopupAsync(const QString &zoneText)
{
    QWidget *parentW = this->window();
    if (!parentW) parentW = this;

    if (m_moveDlg) {
        m_moveDlg->close();
        m_moveDlg = nullptr;
    }

    QDialog *dlg = new QDialog(parentW);
    m_moveDlg = dlg;

    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    dlg->setModal(true);
    dlg->setStyleSheet("QDialog { background: transparent; }");
    dlg->setFixedSize(520, 260);

    const QPoint parentTopLeft = parentW->mapToGlobal(QPoint(0, 0));
    dlg->move(parentTopLeft.x() + (parentW->width() - dlg->width())/2,
              parentTopLeft.y() + (parentW->height()- dlg->height())/2);

    QFrame *card = new QFrame(dlg);
    card->setObjectName("popupCard");
    card->setStyleSheet("#popupCard { background: rgb(246,245,244); border-radius: 16px; }");

    auto *root = new QVBoxLayout(dlg);
    root->setContentsMargins(0,0,0,0);
    root->addWidget(card);

    auto *lay = new QVBoxLayout(card);
    lay->setContentsMargins(28, 24, 28, 24);
    lay->setSpacing(14);

    QLabel *title = new QLabel(QString("%1로 이동 중입니다").arg(zoneText), card);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size:24px; font-weight:900; color:#111827;");

    QLabel *desc = new QLabel("확인을 누르면 이동을 중단합니다", card);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size:16px; font-weight:700; color:#6B7280;");

    QPushButton *ok = new QPushButton("확인(이동 중단)", card);
    ok->setFixedHeight(48);
    ok->setMinimumWidth(260);

    ok->setStyleSheet(
        "QPushButton{ background:#EF4444; color:white; border:none; border-radius:12px;"
        "font-size:18px; font-weight:900; padding:8px 18px; }"
        "QPushButton:hover{ background:#DC2626; }"
        "QPushButton:pressed{ background:#B91C1C; }"
        );

    lay->addStretch();
    lay->addWidget(title);
    lay->addWidget(desc);
    lay->addSpacing(8);
    lay->addWidget(ok, 0, Qt::AlignHCenter);
    lay->addStretch();

    // ✅ 확인 누르면: 이동 취소 신호 + 닫기
    connect(ok, &QPushButton::clicked, this, [this, dlg]() {
        emit requestCancelGoal();
        dlg->close();
    });

    // ✅ 어떤 이유로든 팝업이 닫히면: 이동 취소(안전)
    connect(dlg, &QDialog::finished, this, [this](int){
        emit requestCancelGoal();
        m_moveDlg = nullptr;
    });

    dlg->open(); // ✅ exec() 말고 open()
}

