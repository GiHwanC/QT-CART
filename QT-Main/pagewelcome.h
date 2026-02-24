#ifndef PAGEWELCOME_H
#define PAGEWELCOME_H

#include <QWidget>

class QPropertyAnimation;
class QShowEvent;
class QHideEvent;
class QResizeEvent;

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
    void startRequested();

protected:
    void showEvent(QShowEvent *e) override;
    void hideEvent(QHideEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;

private slots:
    void onStartClicked();

private:
    void loadPixmapsOnce();
    void centerStartImage();          // ✅ 기존: start_image만 가운데
    void centerWelcomeWidgets();      // ✅ 추가: 텍스트/버튼도 가운데
    void resetWelcome();

private:
    Ui::PageWelcome *ui;

    bool m_leaving = false;
    bool m_pixLoaded = false;

    // ✅ 각 위젯의 "기본 Y"를 기억해두고 X만 중앙정렬
    int m_baseYStartImage = -1;
    int m_baseYTitle      = -1;   // qcart_text
    int m_baseYStartText  = -1;   // start_text
    int m_baseYStartBtn   = -1;   // start_button

    QPropertyAnimation *m_anim = nullptr;
};

#endif // PAGEWELCOME_H
