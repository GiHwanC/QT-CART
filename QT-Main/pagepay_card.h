#ifndef PAGEPAY_CARD_H
#define PAGEPAY_CARD_H

#include <QWidget>

namespace Ui {
class pagepay_card;
}

class QShowEvent;

class pagepay_card : public QWidget
{
    Q_OBJECT
public:
    explicit pagepay_card(QWidget *parent = nullptr);
    ~pagepay_card();

signals:
    void goTotalPayClicked();

protected:
    void showEvent(QShowEvent *e) override;

private:
    void showPaymentModal();

private:
    Ui::pagepay_card *ui;
};

#endif // PAGEPAY_CARD_H
