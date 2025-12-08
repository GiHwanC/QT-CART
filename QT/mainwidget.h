#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include "pagewelcome.h"
#include "pagecart.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private:
    Ui::MainWidget *ui;
    PageWelcome *pPageWelcome;
    PageCart *pPageCart;

private slots:
    void on_pPBStartClicked();

};
#endif // MAINWIDGET_H
