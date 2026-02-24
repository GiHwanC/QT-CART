#ifndef PAGEPAY_H
#define PAGEPAY_H

#include <QWidget>
#include <QVector>
#include <QString>

namespace Ui { class PagePay; }

class PagePay : public QWidget
{
    Q_OBJECT
public:
    explicit PagePay(QWidget *parent = nullptr);
    ~PagePay();

    struct PayLine {
        QString name;
        int qty = 0;
        int unitPrice = 0;

    };

    void setPayItems(const QVector<PayLine>& lines);

signals:
    void backCartClicked();      // <- 이전 (PageCart로)
    void creditCardClicked();    // 다음 (pagepay_card로)

private slots:
    void onBackPressed();
    void onNextPressed();

private:
    void setupUiTable();
    void refreshTableAndTotal();

private:
    Ui::PagePay *ui;
    QVector<PayLine> m_lines;
};

#endif // PAGEPAY_H
