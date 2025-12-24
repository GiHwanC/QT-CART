#ifndef PAGEPAY_H
#define PAGEPAY_H

#include <QWidget>

namespace Ui {
class PagePay;
}

class PagePay : public QWidget
{
    Q_OBJECT

public:
    explicit PagePay(QWidget *parent = nullptr);
    ~PagePay();

private:
    Ui::PagePay *ui;
};

#endif // PAGEPAY_H
