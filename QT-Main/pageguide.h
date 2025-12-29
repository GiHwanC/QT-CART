#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>

namespace Ui {
class PageGuide;
}

class PageGuide : public QWidget
{
    Q_OBJECT

public:
    explicit PageGuide(QWidget *parent = nullptr);
    ~PageGuide();

signals:
    void backToCartClicked();
    void requestGoal(double x, double y);

private slots:
    void on_btnBackToCart_clicked();
    void on_foodIcon_clicked();
    void on_groceryIcon_clicked();

private:
    Ui::PageGuide *ui;
};

#endif // PAGEGUIDE_H
