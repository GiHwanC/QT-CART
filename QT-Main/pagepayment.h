#ifndef PAGEPAYMENT_H
#define PAGEPAYMENT_H

#include <QWidget>

namespace Ui {
class pagepayment;
}

class pagepayment : public QWidget
{
    Q_OBJECT

public:
    explicit pagepayment(QWidget *parent = nullptr);
    ~pagepayment();

private:
    Ui::pagepayment *ui;
};

#endif // PAGEPAYMENT_H
