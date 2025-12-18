#ifndef PAGEPAY_CARD_H
#define PAGEPAY_CARD_H

#include <QWidget>

namespace Ui {
class pagepay_card;
}

class pagepay_card : public QWidget
{
    Q_OBJECT

public:
    explicit pagepay_card(QWidget *parent = nullptr);
    ~pagepay_card();

private:
    Ui::pagepay_card *ui;
};

#endif // PAGEPAY_CARD_H
