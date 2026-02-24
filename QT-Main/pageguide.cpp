#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>
#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QResizeEvent>
#include <QDialog>
#include <QLabel>
#include <QGraphicsDropShadowEffect>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);
    
    QPixmap pm(":/etc/guide_map.png");
    if(!pm.isNull()) {
        m_treePixmap = pm; 
        ui->lblmap->setPixmap(pm);
        ui->lblmap->setScaledContents(true);
    }
    ui->lblmap->setStyleSheet("border-radius:14px; background:#ffffff; border: none;");

    // 버튼 연결
    connect(ui->btnBackToCart, &QPushButton::clicked, this, &PageGuide::onBackToCartClicked);
    connect(ui->btnpuzzle, &QPushButton::clicked, this, &PageGuide::onPuzzleClicked);
    connect(ui->btncream,  &QPushButton::clicked, this, &PageGuide::onCreamClicked);
    connect(ui->btnsnack,  &QPushButton::clicked, this, &PageGuide::onSnackClicked);
    connect(ui->btnpay,    &QPushButton::clicked, this, &PageGuide::onPayClicked);
    connect(ui->btnphone,  &QPushButton::clicked, this, &PageGuide::onbtnphoneClicked);
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
    QSize target = ui->lblmap->size();
    QPixmap scaled = m_treePixmap.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui->lblmap->setPixmap(scaled);
}

void PageGuide::onBackToCartClicked()
{
    emit backToCartClicked(); 
}

void PageGuide::onPuzzleClicked()
{
    emit requestGoal(-0.1, -2.4);
    showMovePopup("01코너 식품");
}

void PageGuide::onSnackClicked()
{
    emit requestGoal(3.0, 0.55); 
    showMovePopup("02코너 장난감");
}

void PageGuide::onCreamClicked()
{
    emit requestGoal(2.6, -6.5); 
    showMovePopup("03코너 생필품");
}

void PageGuide::onbtnphoneClicked()
{
    emit requestGoal(-0.1, -5.0); 
    showMovePopup("04코너 전자제품");
}

void PageGuide::onPayClicked()
{
    emit requestGoal(-0.07, -6.7); 
    showMovePopup("결제 구역");
}

void PageGuide::showMovePopup(const QString &zoneText)
{
    // 이미 떠있는 팝업이 있다면 닫기 
    if (m_currentPopup) {
        m_currentPopup->close();
        delete m_currentPopup; 
        m_currentPopup = nullptr;
    }

    QWidget *parentW = this->window();
    m_currentPopup = new QDialog(parentW);
    QDialog &dlg = *m_currentPopup;

    dlg.setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    dlg.setModal(true);
    dlg.setAttribute(Qt::WA_DeleteOnClose); 
    dlg.setStyleSheet("QDialog { background: transparent; }");
    dlg.setFixedSize(520, 260);

    // 팝업이 닫힐 때 포인터 초기화
    connect(&dlg, &QDialog::destroyed, this, [this](){
        m_currentPopup = nullptr;
    });

    // 중앙 정렬 
    const QPoint parentTopLeft = parentW->mapToGlobal(QPoint(0, 0));
    const int x = parentTopLeft.x() + (parentW->width()  - dlg.width())  / 2;
    const int y = parentTopLeft.y() + (parentW->height() - dlg.height()) / 2;
    dlg.move(x, y);

    // 카드 스타일 UI 구성 
    QFrame *card = new QFrame(&dlg);
    card->setObjectName("popupCard");
    card->setStyleSheet("#popupCard { background: rgb(246, 245, 244); border-radius: 16px; }");

    auto *shadow = new QGraphicsDropShadowEffect(card);
    shadow->setBlurRadius(30);
    shadow->setOffset(0, 8);
    shadow->setColor(QColor(0, 0, 0, 80));
    card->setGraphicsEffect(shadow);

    QVBoxLayout *root = new QVBoxLayout(&dlg);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(card);

    QVBoxLayout *lay = new QVBoxLayout(card);
    lay->setContentsMargins(28, 24, 28, 24);
    lay->setSpacing(14);

    QLabel *title = new QLabel(QString("%1로 이동 중입니다").arg(zoneText), card);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size:24px; font-weight:900; color:#111827;");

    QLabel *desc = new QLabel("확인을 누르면 이동을 중단합니다.", card);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size:16px; font-weight:700; color:#6B7280;");

    QPushButton *stopBtn = new QPushButton("확인(이동 중단)", card);
    stopBtn->setFixedHeight(48);
    stopBtn->setMinimumWidth(220);
    stopBtn->setStyleSheet(
        "QPushButton{ background:#EF4444; color:white; border:none; border-radius:12px; " 
        "font-size:18px; font-weight:900; padding:8px 18px; }"
        "QPushButton:hover{ background:#DC2626; }"
        "QPushButton:pressed{ background:#B91C1C; }"
    );

    lay->addStretch();
    lay->addWidget(title);
    lay->addWidget(desc);
    lay->addSpacing(8);
    lay->addWidget(stopBtn, 0, Qt::AlignHCenter);
    lay->addStretch();

    // 안내 중지 버튼 -> MainWidget에 신호
    connect(stopBtn, &QPushButton::clicked, this, [this, &dlg](){
        emit requestStop(); 
        dlg.accept();
    });

    dlg.show(); 
}

// 도착 알림 -> 팝업 끄기
void PageGuide::onRobotArrived()
{
    if (m_currentPopup) {
        qDebug() << "PageGuide: Robot Arrived! Closing popup.";
        m_currentPopup->accept(); 
    }
}
