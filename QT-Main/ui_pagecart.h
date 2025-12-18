/********************************************************************************
** Form generated from reading UI file 'pagecart.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGECART_H
#define UI_PAGECART_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PageCart
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *labelTitle;
    QLabel *label_2;
    QPushButton *pushButton;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QLabel *label;
    QTableWidget *tableCart;
    QGridLayout *gridLayout;
    QLabel *labelTotalPriceValue;
    QPushButton *btnPay;
    QLabel *labelTotalPrice;
    QPushButton *btnGuideMode;
    QLabel *label_5;
    QLabel *label_4;

    void setupUi(QWidget *PageCart)
    {
        if (PageCart->objectName().isEmpty())
            PageCart->setObjectName("PageCart");
        PageCart->resize(770, 672);
        verticalLayout_2 = new QVBoxLayout(PageCart);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        horizontalLayout_4->setContentsMargins(-1, 0, -1, -1);
        labelTitle = new QLabel(PageCart);
        labelTitle->setObjectName("labelTitle");
        labelTitle->setStyleSheet(QString::fromUtf8("color: rgb(53, 132, 228);\n"
"font: 700 20pt \"Ria Sans\";"));

        horizontalLayout_4->addWidget(labelTitle);

        label_2 = new QLabel(PageCart);
        label_2->setObjectName("label_2");
        label_2->setStyleSheet(QString::fromUtf8("font: 700 18pt \"Ria Sans\";\n"
"color: rgb(0, 0, 0);"));

        horizontalLayout_4->addWidget(label_2);

        pushButton = new QPushButton(PageCart);
        pushButton->setObjectName("pushButton");
        pushButton->setMinimumSize(QSize(0, 0));
        pushButton->setStyleSheet(QString::fromUtf8("font: 20pt \"Ria Sans\";\n"
"    border-radius: 5px;\n"
"      \n"
""));

        horizontalLayout_4->addWidget(pushButton);

        horizontalLayout_4->setStretch(1, 1);

        horizontalLayout->addLayout(horizontalLayout_4);


        verticalLayout->addLayout(horizontalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName("verticalLayout_5");
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(10);
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        horizontalLayout_2->setContentsMargins(-1, 0, -1, 0);
        label_3 = new QLabel(PageCart);
        label_3->setObjectName("label_3");
        label_3->setStyleSheet(QString::fromUtf8("font: 20pt \"Ria Sans\";\n"
"\n"
"color: rgb(94, 92, 100);"));

        horizontalLayout_2->addWidget(label_3);

        label = new QLabel(PageCart);
        label->setObjectName("label");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMinimumSize(QSize(33, 148));
        label->setMaximumSize(QSize(720, 720));
        label->setBaseSize(QSize(0, 0));
        label->setStyleSheet(QString::fromUtf8("image: url(:/new/prefix1/Gemini_Generated_Image_u34sf6u34sf6u34s.png);"));

        horizontalLayout_2->addWidget(label);

        horizontalLayout_2->setStretch(1, 1);

        verticalLayout_5->addLayout(horizontalLayout_2);


        verticalLayout_3->addLayout(verticalLayout_5);

        tableCart = new QTableWidget(PageCart);
        if (tableCart->columnCount() < 6)
            tableCart->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QFont font;
        font.setFamilies({QString::fromUtf8("Ria Sans")});
        font.setItalic(true);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        __qtablewidgetitem1->setFont(font);
        tableCart->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        __qtablewidgetitem2->setFont(font);
        tableCart->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        __qtablewidgetitem5->setFont(font);
        tableCart->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        tableCart->setObjectName("tableCart");
        tableCart->setFrameShape(QFrame::Shape::Box);

        verticalLayout_3->addWidget(tableCart);

        verticalLayout_3->setStretch(0, 1);
        verticalLayout_3->setStretch(1, 3);

        verticalLayout->addLayout(verticalLayout_3);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(0);
        gridLayout->setObjectName("gridLayout");
        labelTotalPriceValue = new QLabel(PageCart);
        labelTotalPriceValue->setObjectName("labelTotalPriceValue");
        labelTotalPriceValue->setStyleSheet(QString::fromUtf8("font: 30pt \"Ria Sans\";\n"
"background-color: rgb(119, 118, 123);\n"
"color: rgb(255, 255, 255);"));

        gridLayout->addWidget(labelTotalPriceValue, 1, 2, 1, 1);

        btnPay = new QPushButton(PageCart);
        btnPay->setObjectName("btnPay");
        btnPay->setMinimumSize(QSize(100, 141));
        btnPay->setStyleSheet(QString::fromUtf8("font: 40pt \"Ria Sans\";\n"
"\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(224, 27, 36);\n"
"\n"
"    border-radius: 5px;\n"
"    padding:0px 0px;\n"
"   \n"
"\n"
"hover {\n"
"    background-color: rgb(246, 97, 81)\n"
"}\n"
"\n"
"       \n"
"          \n"
"       \n"
"\n"
"       \n"
"               \n"
"\n"
"\n"
"       "));

        gridLayout->addWidget(btnPay, 0, 3, 1, 1);

        labelTotalPrice = new QLabel(PageCart);
        labelTotalPrice->setObjectName("labelTotalPrice");
        labelTotalPrice->setMaximumSize(QSize(300, 169));
        labelTotalPrice->setStyleSheet(QString::fromUtf8("font: 30pt \"Ria Sans\";\n"
"background-color: rgb(119, 118, 123);\n"
"color: rgb(255, 255, 255);\n"
""));

        gridLayout->addWidget(labelTotalPrice, 0, 1, 1, 1);

        btnGuideMode = new QPushButton(PageCart);
        btnGuideMode->setObjectName("btnGuideMode");
        btnGuideMode->setMinimumSize(QSize(145, 56));
        btnGuideMode->setStyleSheet(QString::fromUtf8("font: 23pt \"Ria Sans\";\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(224, 27, 36);\n"
"\n"
"    border-radius: 0px;\n"
"    padding: 0px 0px;\n"
"   \n"
"\n"
"hover {\n"
"    background-color: rgb(53, 132, 228)\n"
"}\n"
"\n"
"       \n"
"          \n"
"       \n"
""));
        btnGuideMode->setIconSize(QSize(28, 80));

        gridLayout->addWidget(btnGuideMode, 1, 3, 1, 1);

        label_5 = new QLabel(PageCart);
        label_5->setObjectName("label_5");
        label_5->setMinimumSize(QSize(0, 0));
        label_5->setStyleSheet(QString::fromUtf8("background-color: rgb(119, 118, 123);\n"
"color: rgb(255, 255, 255);\n"
""));

        gridLayout->addWidget(label_5, 0, 2, 1, 1);

        label_4 = new QLabel(PageCart);
        label_4->setObjectName("label_4");
        label_4->setStyleSheet(QString::fromUtf8("background-color: rgb(119, 118, 123);\n"
"color: rgb(255, 255, 255);\n"
""));

        gridLayout->addWidget(label_4, 1, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageCart);

        QMetaObject::connectSlotsByName(PageCart);
    } // setupUi

    void retranslateUi(QWidget *PageCart)
    {
        PageCart->setWindowTitle(QCoreApplication::translate("PageCart", "Form", nullptr));
        labelTitle->setText(QCoreApplication::translate("PageCart", "QT", nullptr));
        label_2->setText(QCoreApplication::translate("PageCart", "CART", nullptr));
        pushButton->setText(QCoreApplication::translate("PageCart", "\360\237\217\240\342\200\213", nullptr));
        label_3->setText(QCoreApplication::translate("PageCart", "\352\265\254\353\247\244\355\225\230\354\213\244 \354\203\201\355\222\210\354\235\204 \354\212\244\354\272\224\355\225\264\354\243\274\354\204\270\354\232\224", nullptr));
        label->setText(QString());
        QTableWidgetItem *___qtablewidgetitem = tableCart->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("PageCart", "\354\203\201\355\222\210\353\252\205", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = tableCart->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("PageCart", "\352\260\234\354\210\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = tableCart->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("PageCart", "+", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = tableCart->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("PageCart", "-", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = tableCart->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("PageCart", "\352\270\210\354\225\241", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = tableCart->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("PageCart", "\354\202\255\354\240\234", nullptr));
        labelTotalPriceValue->setText(QCoreApplication::translate("PageCart", "                           0\354\233\220", nullptr));
        btnPay->setText(QCoreApplication::translate("PageCart", "\352\262\260\354\240\234", nullptr));
        labelTotalPrice->setText(QCoreApplication::translate("PageCart", "\352\262\260\354\240\234\352\270\210\354\225\241", nullptr));
        btnGuideMode->setText(QCoreApplication::translate("PageCart", "\354\203\201\355\222\210 \354\260\276\352\270\260\360\237\224\215", nullptr));
        label_5->setText(QString());
        label_4->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class PageCart: public Ui_PageCart {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGECART_H
