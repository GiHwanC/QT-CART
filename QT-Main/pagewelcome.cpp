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

    // ✅ UI 레이아웃 적용된 다음 기준 Y 저장 + 중앙 정렬
    QTimer::singleShot(0, this, [this]() {
        if (ui->start_image && m_baseY < 0) {
            m_baseY = ui->start_image->y(); // UI에서 잡힌 기본 Y
        }
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

    // ✅ cart_image / wind_image pixmap 강제 세팅
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

    // ✅ 페이지가 숨겨질 때 애니메이션이 남아있으면 정리(안전)
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
        centerStartImage();
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

    // ✅ start_image 다시 보이기 + 가운데로 복귀
    if (ui->start_image) ui->start_image->show();
    if (ui->cart_image)  ui->cart_image->show();
    if (ui->wind_image)  ui->wind_image->show();

    // 레이아웃 반영 후 이동
    QTimer::singleShot(0, this, [this]() {
        // m_baseY가 아직이면 지금 값으로 잡기
        if (ui->start_image && m_baseY < 0) m_baseY = ui->start_image->y();
        centerStartImage();
    });
}

void PageWelcome::centerStartImage()
{
    if (!ui->start_image) return;

    QWidget *parentW = ui->start_image->parentWidget();
    if (!parentW) parentW = this;

    const int pw = parentW->width();
    const int w  = ui->start_image->width();

    const int x = (pw - w) / 2;

    // ✅ Y는 처음 UI에서 잡힌 위치 유지(원하면 세로 중앙으로 바꿔도 됨)
    int y = (m_baseY >= 0) ? m_baseY : ui->start_image->y();

    ui->start_image->move(x, y);
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
        // 애니메이션 포인터 정리
        if (m_anim) {
            m_anim->deleteLater();
            m_anim = nullptr;
        }

        // ✅ MainWidget에서 PageCart로 전환되게 신호 쏴주기(둘 다 유지)
        emit startRequested();
        emit startClicked();
    });

    m_anim->start();
}
