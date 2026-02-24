#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>
#include <QPixmap>
#include <QDialog> 

namespace Ui { class PageGuide; }

class PageGuide : public QWidget
{
    Q_OBJECT

public:
    explicit PageGuide(QWidget *parent = nullptr);
    ~PageGuide();

public slots:
    void onRobotArrived();

signals:
    void backToCartClicked();
    void requestGoal(double x, double y);
    void requestStop();

protected:
    void resizeEvent(QResizeEvent *e) override;

private slots:
    void onBackToCartClicked();
    void onPuzzleClicked();
    void onCreamClicked();
    void onSnackClicked();
    void onPayClicked();
    void onbtnphoneClicked();

private:
    Ui::PageGuide *ui;
    QPixmap m_treePixmap;
    QDialog *m_currentPopup = nullptr; 

    void applyTreePixmap();
    void showMovePopup(const QString &zoneText);                   
};

#endif // PAGEGUIDE_H