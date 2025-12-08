#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>

namespace Ui {
class PageCart;
}

class PageCart : public QWidget
{
    Q_OBJECT

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();

private:
    Ui::PageCart *ui;
};

#endif // PAGECART_H
