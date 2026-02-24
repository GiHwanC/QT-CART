#ifndef PAGETOTALPAY_H
#define PAGETOTALPAY_H

#include <QWidget>
#include <QPixmap>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsProxyWidget;
class QVariantAnimation;

namespace Ui {
class PageTotalPay;
}

class PageTotalPay : public QWidget
{
    Q_OBJECT
public:
    explicit PageTotalPay(QWidget *parent = nullptr);
    ~PageTotalPay();

signals:
    void backToStartClicked();   // ✅ 어디든 클릭하면 Welcome으로 돌아가기

protected:
    void resizeEvent(QResizeEvent *e) override;
    bool eventFilter(QObject *obj, QEvent *event) override;
    void mousePressEvent(QMouseEvent *e) override;

private:
    QPixmap makeCheckIcon(int sizePx) const;
    void centerProxy();

private:
    Ui::PageTotalPay *ui = nullptr;

    QGraphicsView *m_view = nullptr;
    QGraphicsScene *m_scene = nullptr;
    QGraphicsProxyWidget *m_proxy = nullptr;

    QWidget *m_content = nullptr;
    QVariantAnimation *m_pulseAnim = nullptr;
};

#endif // PAGETOTALPAY_H
