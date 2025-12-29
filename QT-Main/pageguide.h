#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>
#include <QHideEvent>

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

protected:
    // 다른 화면으로 갈 때 자동 실행되는 이벤트 함수
    void hideEvent(QHideEvent *event) override;

private:
    Ui::PageGuide *ui;
};

#endif // PAGEGUIDE_H
