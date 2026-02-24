#include "pagetotalpay.h"
#include "ui_pagetotalpay.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QVariantAnimation>

#include <QVBoxLayout>
#include <QLabel>
#include <QEvent>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QTimer>

#include <QtMath>
#include <QPainter>
#include <QPen>
#include <QEasingCurve>

PageTotalPay::PageTotalPay(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageTotalPay)
{
    ui->setupUi(this);

    if (objectName().isEmpty())
        setObjectName("PageTotalPay");

    // ✅ 여기 추가
    setStyleSheet("QWidget#PageTotalPay { background-color: rgb(34,197,94); }");


    // ✅ Scene/View
    m_scene = new QGraphicsScene(this);

    m_view = new QGraphicsView(this);
    m_view->setScene(m_scene);
    m_view->setFrameShape(QFrame::NoFrame);
    m_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_view->setStyleSheet("background: transparent;");
    m_view->setAlignment(Qt::AlignCenter);

    // 클릭 이벤트를 view/viewport에서도 잡기
    m_view->viewport()->installEventFilter(this);
    m_view->installEventFilter(this);

    // ✅ content 위젯(아이콘+텍스트)
    m_content = new QWidget();
    m_content->setAttribute(Qt::WA_TranslucentBackground);
    m_content->installEventFilter(this);

    auto *lay = new QVBoxLayout(m_content);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(28);
    lay->setAlignment(Qt::AlignCenter);

    QLabel *lblIcon = new QLabel(m_content);
    lblIcon->setFixedSize(140, 140);
    lblIcon->setAlignment(Qt::AlignCenter);
    lblIcon->setPixmap(makeCheckIcon(128));
    lblIcon->setStyleSheet("background: transparent;");

    QLabel *lblTitle = new QLabel("구매해주셔서 감사합니다", m_content);
    lblTitle->setAlignment(Qt::AlignCenter);
    lblTitle->setStyleSheet(
        "background: transparent;"
        "color: rgb(0,0,0);"
        "font-weight: 900;"
        "font-size: 44px;"
        );

    QLabel *lblSub = new QLabel("화면을 다시 눌러 시작하기", m_content);
    lblSub->setAlignment(Qt::AlignCenter);
    lblSub->setStyleSheet(
        "background: transparent;"
        "color: rgb(0,0,0);"
        "font-weight: 700;"
        "font-size: 22px;"
        );

    lay->addWidget(lblIcon, 0, Qt::AlignCenter);
    lay->addWidget(lblTitle, 0, Qt::AlignCenter);
    lay->addWidget(lblSub, 0, Qt::AlignCenter);

    // ✅ proxy로 올리기(스케일 애니메이션 대상)
    m_proxy = m_scene->addWidget(m_content);
    m_proxy->setTransformOriginPoint(m_proxy->boundingRect().center());

    // ✅ 레이아웃 붙이기 (중요: 기존 레이아웃 있으면 거기에 붙임)
    if (this->layout()) {
        this->layout()->setContentsMargins(0,0,0,0);
        this->layout()->addWidget(m_view);
    } else {
        auto *root = new QVBoxLayout(this);
        root->setContentsMargins(0,0,0,0);
        root->addWidget(m_view);
    }

    // ✅❗️여기가 너 코드에서 크래시 난 부분: m_pulseAnim을 반드시 생성해야 함
    m_pulseAnim = new QVariantAnimation(this);

    // ✅ 코사인 루프 (자연스럽게 1.0 -> 1.15 -> 1.0, 점프 없음)
    m_pulseAnim->setStartValue(0.0);
    m_pulseAnim->setEndValue(2.0 * M_PI);
    m_pulseAnim->setDuration(2000);
    m_pulseAnim->setLoopCount(-1);
    m_pulseAnim->setEasingCurve(QEasingCurve::Linear);

    connect(m_pulseAnim, &QVariantAnimation::valueChanged, this, [this](const QVariant &v){
        const qreal t = v.toReal();
        const qreal amp = 0.15;
        const qreal scale = 1.0 + amp * 0.5 * (1.0 - qCos(t));
        if (m_proxy) m_proxy->setScale(scale);
    });

    m_pulseAnim->start();

    // ✅ viewport size가 0일 수 있으니 show 이후에 중앙 정렬
    QTimer::singleShot(0, this, [this](){ centerProxy(); });
}

PageTotalPay::~PageTotalPay()
{
    delete ui;
}

QPixmap PageTotalPay::makeCheckIcon(int sizePx) const
{
    const int pad = 6;
    QPixmap pm(sizePx + pad*2, sizePx + pad*2);
    pm.fill(Qt::transparent);

    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);

    // ✅ 초록색 (Tailwind green-500 비슷)
    QColor green(34, 197, 94);

    // circle stroke
    QPen pen(green);                 // ✅ 여기!
    pen.setWidthF(sizePx * 0.07);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);

    QRectF rc(pad, pad, sizePx, sizePx);
    p.drawEllipse(rc);

    // check
    p.setPen(Qt::NoPen);
    p.setBrush(green);
    p.drawEllipse(rc);

    // 체크: 흰색
    QPen pen2(Qt::white);
    pen2.setWidthF(sizePx * 0.09);
    pen2.setCapStyle(Qt::RoundCap);
    pen2.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen2);
    p.setBrush(Qt::NoBrush);

    QPointF a(rc.left() + sizePx*0.28, rc.top() + sizePx*0.55);
    QPointF b(rc.left() + sizePx*0.46, rc.top() + sizePx*0.70);
    QPointF c(rc.left() + sizePx*0.74, rc.top() + sizePx*0.36);

    p.drawLine(a, b);
    p.drawLine(b, c);

    return pm;
}


void PageTotalPay::centerProxy()
{
    if (!m_view || !m_scene || !m_proxy) return;

    const QSize vsz = m_view->viewport()->size();
    if (vsz.width() <= 0 || vsz.height() <= 0) return; // ✅ 0이면 아직 표시 전

    m_scene->setSceneRect(0, 0, vsz.width(), vsz.height());

    const QPointF origin = m_proxy->boundingRect().center();
    m_proxy->setTransformOriginPoint(origin);

    const QPointF sceneCenter = m_scene->sceneRect().center();
    m_proxy->setPos(sceneCenter - origin);
}

void PageTotalPay::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);
    centerProxy();
}

bool PageTotalPay::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::MouseButtonPress) {
        emit backToStartClicked();
        return true;
    }
    return QWidget::eventFilter(obj, event);
}

void PageTotalPay::mousePressEvent(QMouseEvent *e)
{
    emit backToStartClicked();
    QWidget::mousePressEvent(e);
}
