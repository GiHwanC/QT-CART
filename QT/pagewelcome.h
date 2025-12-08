#ifndef PAGEWELCOME_H
#define PAGEWELCOME_H

#include <QWidget>

namespace Ui {
class PageWelcome;
}

class PageWelcome : public QWidget
{
    Q_OBJECT

public:
    explicit PageWelcome(QWidget *parent = nullptr);
    ~PageWelcome();

signals:
    void startClicked();

private:
    Ui::PageWelcome *ui;
};

#endif // PAGEWELCOME_H
