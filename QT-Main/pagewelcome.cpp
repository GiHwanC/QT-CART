#include "pagewelcome.h"
#include "ui_pagewelcome.h"

#include <QPushButton>
#include <QPropertyAnimation>
#include <QEasingCurve>
#include <QTimer>
#include <QPixmap>
#include <QDebug>
#include <QShowEvent>
#include <QHideEvent>
#include <QResizeEvent>

PageWelcome::PageWelcome(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageWelcome)
{
    ui->setupUi(this);

    // ✅ 버튼 연결
    connect(ui->start_button, &QPushButton::clicked,
            this, &PageWelcome::onStartClicked);

    // ✅ 이미지 로드/세팅(한 번만)
    loadPixmapsOnce();

    // ✅ UI 레이아웃 적용된 다음 기본 Y 저장 + 중앙 정렬
    QTimer::singleShot(0, this, [this]() {

        if (ui->start_image && m_baseYStartImage < 0)
            m_baseYStartImage = ui->start_image->y();

        if (ui->qtcart_text && m_baseYTitle < 0)
            m_baseYTitle = ui->qtcart_text->y();

        if (ui->start_text && m_baseYStartText < 0)
            m_baseYStartText = ui->start_text->y();

        if (ui->start_button && m_baseYStartBtn < 0)
            m_baseYStartBtn = ui->start_button->y();

        resetWelcome();
    });
}

PageWelcome::~PageWelcome()
{
    if (m_anim) {
        m_anim->stop();
        delete m_anim;
        m_anim = nullptr;
    }
    delete ui;
}

void PageWelcome::loadPixmapsOnce()
{
    if (m_pixLoaded) return;
    m_pixLoaded = true;

    QPixmap cartPx(":/etc/cart.png");
    QPixmap windPx(":/etc/wind.png");

    qDebug() << "[cartPx isNull?]" << cartPx.isNull();
    qDebug() << "[windPx isNull?]" << windPx.isNull();

    if (ui->cart_image) {
        ui->cart_image->setPixmap(cartPx);
        ui->cart_image->setScaledContents(true);
        ui->cart_image->setAttribute(Qt::WA_TranslucentBackground);
        ui->cart_image->show();
    }

    if (ui->wind_image) {
        ui->wind_image->setPixmap(windPx);
        ui->wind_image->setScaledContents(true);
        ui->wind_image->setAttribute(Qt::WA_TranslucentBackground);
        ui->wind_image->show();
    }

    if (ui->start_image) ui->start_image->show();
}

void PageWelcome::showEvent(QShowEvent *e)
{
    QWidget::showEvent(e);

    // ✅ 다른 페이지 갔다가 다시 돌아왔을 때: 항상 초기화
    QTimer::singleShot(0, this, [this]() {
        resetWelcome();
    });
}

void PageWelcome::hideEvent(QHideEvent *e)
{
    QWidget::hideEvent(e);

    // ✅ 페이지가 숨겨질 때 애니메이션 남아있으면 정리(안전)
    if (m_anim) {
        m_anim->stop();
        m_anim->deleteLater();
        m_anim = nullptr;
    }
}

void PageWelcome::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);

    // ✅ 떠나지 않은 상태면 리사이즈 때도 가운데 유지
    if (!m_leaving) {
        centerStartImage();        // ✅ 기존 유지
        centerWelcomeWidgets();    // ✅ 추가
    }
}

void PageWelcome::resetWelcome()
{
    // ✅ 상태 초기화
    m_leaving = false;

    // ✅ 버튼 다시 누르게
    if (ui->start_button) {
        ui->start_button->setEnabled(true);
    }

    // ✅ 애니메이션 남아있으면 정리
    if (m_anim) {
        m_anim->stop();
        m_anim->deleteLater();
        m_anim = nullptr;
    }

    // ✅ 위젯 다시 보이기
    if (ui->start_image) ui->start_image->show();
    if (ui->cart_image)  ui->cart_image->show();
    if (ui->wind_image)  ui->wind_image->show();

    // 레이아웃 반영 후 이동
    QTimer::singleShot(0, this, [this]() {
        if (ui->start_image && m_baseYStartImage < 0) m_baseYStartImage = ui->start_image->y();
        if (ui->qtcart_text  && m_baseYTitle < 0)      m_baseYTitle = ui->qtcart_text->y();
        if (ui->start_text  && m_baseYStartText < 0)  m_baseYStartText = ui->start_text->y();
        if (ui->start_button&& m_baseYStartBtn < 0)   m_baseYStartBtn = ui->start_button->y();

        centerStartImage();        // ✅ 기존 유지
        centerWelcomeWidgets();    // ✅ 추가
    });
}

// ✅ 기존 함수는 그대로: start_image만 가운데
void PageWelcome::centerStartImage()
{
    if (!ui->start_image) return;

    QWidget *parentW = ui->start_image->parentWidget();
    if (!parentW) parentW = this;

    const int pw = parentW->width();
    const int w  = ui->start_image->width();
    const int x  = (pw - w) / 2;

    int y = (m_baseYStartImage >= 0) ? m_baseYStartImage : ui->start_image->y();

    ui->start_image->move(x, y);
}

// ✅ 추가: qcart_text, start_text, start_button도 “X만” 중앙정렬
void PageWelcome::centerWelcomeWidgets()
{
    QWidget *parentW = this; // 보통 PageWelcome 전체 기준이 가장 깔끔함
    const int pw = parentW->width();

    auto centerXKeepY = [&](QWidget *w, int baseY){
        if (!w) return;
        const int x = (pw - w->width()) / 2;
        const int y = (baseY >= 0) ? baseY : w->y();
        w->move(x, y);
    };

    centerXKeepY(ui->qtcart_text,    m_baseYTitle);
    centerXKeepY(ui->start_text,    m_baseYStartText);
    centerXKeepY(ui->start_button,  m_baseYStartBtn);
}

void PageWelcome::onStartClicked()
{
    if (m_leaving) return;
    m_leaving = true;

    if (ui->start_button)
        ui->start_button->setEnabled(false);

    if (!ui->start_image) {
        emit startRequested();
        emit startClicked();
        return;
    }

    QWidget *parentW = ui->start_image->parentWidget();
    if (!parentW) parentW = this;

    const QPoint startPos = ui->start_image->pos();
    const int y = startPos.y();

    // ✅ 오른쪽 화면 밖으로 슈웅~
    const int endX = parentW->width() + ui->start_image->width() + 20;
    const QPoint endPos(endX, y);

    // ✅ 멤버로 애니메이션 보관(돌아올 때 stop/정리 가능)
    m_anim = new QPropertyAnimation(ui->start_image, "pos", this);
    m_anim->setDuration(700);
    m_anim->setEasingCurve(QEasingCurve::InOutCubic);
    m_anim->setStartValue(startPos);
    m_anim->setEndValue(endPos);

    connect(m_anim, &QPropertyAnimation::finished, this, [this]() {
        if (m_anim) {
            m_anim->deleteLater();
            m_anim = nullptr;
        }
        emit startRequested();
        emit startClicked();
    });

    m_anim->start();
}
