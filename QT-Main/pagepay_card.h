#ifndef PAGEPAY_CARD_H
#define PAGEPAY_CARD_H

#include <QWidget>
#include <QPixmap>

namespace Ui {
class pagepay_card;
}
class QShowEvent;
class QResizeEvent;

class pagepay_card : public QWidget
{
    Q_OBJECT
public:
    explicit pagepay_card(QWidget *parent = nullptr);
    ~pagepay_card();

signals:
    void goTotalPayClicked();   // ✅ 확인 누르면 pagetotalpay로 이동

protected:
    void showEvent(QShowEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;

private:
    void showPaymentModal();

private:
    Ui::pagepay_card *ui;
    bool m_modalShown = false;
    // QPixmap makeEmojiPixmap(const QString& emoji, int px) const;
    // QPixmap rotatedPixmap(const QPixmap& src, qreal deg) const;
};

#endif // PAGEPAY_CARD_H
