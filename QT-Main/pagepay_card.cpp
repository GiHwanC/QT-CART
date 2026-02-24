#include "pagepay_card.h"
#include "ui_pagepay_card.h"

#include <QDialog>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QTimer>
#include <QShowEvent>
#include <QPainter>
#include <QFont>
#include <QSharedPointer>

// ⏳ 텍스트를 Pixmap으로 렌더링
static QPixmap textToPixmap(const QString &text, int fontPx)
{
    QFont f;
    f.setPixelSize(fontPx);

    const int size = fontPx * 2;
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);

    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);
    p.setFont(f);
    p.setPen(Qt::black);
    p.drawText(pm.rect(), Qt::AlignCenter, text);
    return pm;
}

// Pixmap을 고정 캔버스 안에서 회전
static QPixmap rotatedPixmap(const QPixmap &src, qreal angleDeg)
{
    QPixmap out(src.size());
    out.fill(Qt::transparent);

    QPainter p(&out);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const QPointF c(out.width() / 2.0, out.height() / 2.0);
    p.translate(c);
    p.rotate(angleDeg);
    p.translate(-c);
    p.drawPixmap(0, 0, src);

    return out;
}

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

void pagepay_card::showEvent(QShowEvent *e)
{
    QWidget::showEvent(e);

    // 페이지가 뜬 뒤 모달 띄우기
    QTimer::singleShot(0, this, [this]() {
        showPaymentModal();
    });
}

void pagepay_card::showPaymentModal()
{
    QWidget *parentW = this->window();
    if (!parentW) parentW = this;

    // ✅ 팝업 자체를 카드 크기로만 만들기 (전체 덮지 않음)
    QDialog *dlg = new QDialog(parentW);
    dlg->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    dlg->setModal(true);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setFixedSize(520, 260);

    // ✅ 부모(800x480) 기준 "정중앙" 위치
    const int x = (parentW->width()  - dlg->width())  / 2;
    const int y = (parentW->height() - dlg->height()) / 2;
    dlg->move(x, y);

    // ===== 다이얼로그 루트 레이아웃 =====
    auto *dlgLay = new QVBoxLayout(dlg);
    dlgLay->setContentsMargins(0,0,0,0);
    dlgLay->setSpacing(0);

    // ===== 카드 프레임 (둥근 모서리) =====
    QFrame *card = new QFrame(dlg);
    card->setObjectName("card");
    card->setStyleSheet(
        "#card { background: white; border-radius: 16px; }"
        );
    dlgLay->addWidget(card);

    // ===== 카드 내부 스택 =====
    QStackedWidget *stack = new QStackedWidget(card);
    QVBoxLayout *cardLay = new QVBoxLayout(card);
    cardLay->setContentsMargins(24,24,24,24);
    cardLay->addWidget(stack);

    // --- processing ---
    QWidget *wProc = new QWidget(card);
    QVBoxLayout *pLay = new QVBoxLayout(wProc);
    pLay->setContentsMargins(0,0,0,0);
    pLay->setSpacing(12);

    QLabel *spinner = new QLabel(wProc);
    spinner->setAlignment(Qt::AlignCenter);
    spinner->setStyleSheet("background: transparent;");

    const QPixmap baseHourglass = textToPixmap("⏳", 52);
    spinner->setPixmap(baseHourglass);

    QLabel *title = new QLabel("카드를 뽑지말아주세요", wProc);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size: 22px; font-weight: 800; color: #111827;");

    QLabel *desc = new QLabel("결제가 진행중입니다", wProc);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size: 14px; color: #6B7280;");

    QLabel *countLbl = new QLabel("5", wProc);
    countLbl->setAlignment(Qt::AlignCenter);
    countLbl->setStyleSheet("font-size: 64px; font-weight: 900; color: #2563EB; margin-top: 6px;");

    pLay->addStretch();
    pLay->addWidget(spinner);
    pLay->addWidget(title);
    pLay->addWidget(desc);
    pLay->addWidget(countLbl);
    pLay->addStretch();

    // --- complete ---
    QWidget *wDone = new QWidget(card);
    QVBoxLayout *dLay = new QVBoxLayout(wDone);
    dLay->setContentsMargins(0,0,0,0);
    dLay->setSpacing(16);

    QLabel *check = new QLabel("✓", wDone);
    check->setAlignment(Qt::AlignCenter);
    check->setStyleSheet("font-size: 72px; font-weight: 900; color: #22C55E;");

    QLabel *doneTxt = new QLabel("결제가 완료되었습니다", wDone);
    doneTxt->setAlignment(Qt::AlignCenter);
    doneTxt->setStyleSheet("font-size: 22px; font-weight: 800; color: #111827;");

    QPushButton *btnOk = new QPushButton("확인", wDone);
    btnOk->setFixedHeight(44);
    btnOk->setFixedWidth(180);
    btnOk->setStyleSheet(
        "QPushButton{background:#2563EB;color:white;border:none;border-radius:12px;font-size:16px;font-weight:800; padding: 0 18px;}"
        "QPushButton:hover{background:#1D4ED8;}"
        "QPushButton:pressed{background:#1E40AF;}"
        );

    dLay->addStretch();
    dLay->addWidget(check);
    dLay->addWidget(doneTxt);
    dLay->addWidget(btnOk, 0, Qt::AlignHCenter);
    dLay->addStretch();

    stack->addWidget(wProc);
    stack->addWidget(wDone);
    stack->setCurrentWidget(wProc);

    // ===== (A) ⏳ 회전 타이머 (느리게) =====
    auto angle = QSharedPointer<qreal>::create(0.0);
    QTimer *spinTimer = new QTimer(dlg);
    spinTimer->setInterval(30); // ✅ 16ms(빠름) -> 30ms(느리게)

    QObject::connect(spinTimer, &QTimer::timeout, dlg, [=]() mutable {
        *angle += 2.0;              // ✅ 4.0(빠름) -> 2.0(느리게)
        if (*angle >= 360.0) *angle -= 360.0;
        spinner->setPixmap(rotatedPixmap(baseHourglass, *angle));
    });
    spinTimer->start();

    // ===== (B) 5초 카운트다운 =====
    auto count = QSharedPointer<int>::create(5);
    QTimer *t = new QTimer(dlg);
    t->setInterval(1000);

    QObject::connect(t, &QTimer::timeout, dlg, [=]() mutable {
        (*count)--;
        if (*count >= 0) countLbl->setText(QString::number(*count));

        if (*count <= 0) {
            t->stop();
            spinTimer->stop();
            stack->setCurrentWidget(wDone);
        }
    });

    QObject::connect(btnOk, &QPushButton::clicked, dlg, [this, dlg]() {
        dlg->accept();
        emit goTotalPayClicked();
    });

    t->start();
    dlg->exec();
}

