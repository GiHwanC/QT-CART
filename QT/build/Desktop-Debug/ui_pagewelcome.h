/********************************************************************************
** Form generated from reading UI file 'pagewelcome.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEWELCOME_H
#define UI_PAGEWELCOME_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PageWelcome
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer;
    QLabel *label;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pPBStart;
    QSpacerItem *verticalSpacer_3;

    void setupUi(QWidget *PageWelcome)
    {
        if (PageWelcome->objectName().isEmpty())
            PageWelcome->setObjectName(QString::fromUtf8("PageWelcome"));
        PageWelcome->resize(612, 416);
        verticalLayout_2 = new QVBoxLayout(PageWelcome);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        label = new QLabel(PageWelcome);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setPointSize(30);
        font.setBold(true);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        pPBStart = new QPushButton(PageWelcome);
        pPBStart->setObjectName(QString::fromUtf8("pPBStart"));
        pPBStart->setIconSize(QSize(250, 250));
        pPBStart->setCheckable(false);

        verticalLayout->addWidget(pPBStart);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        verticalLayout->setStretch(0, 1);
        verticalLayout->setStretch(1, 4);
        verticalLayout->setStretch(2, 1);
        verticalLayout->setStretch(3, 4);
        verticalLayout->setStretch(4, 1);

        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageWelcome);

        QMetaObject::connectSlotsByName(PageWelcome);
    } // setupUi

    void retranslateUi(QWidget *PageWelcome)
    {
        PageWelcome->setWindowTitle(QCoreApplication::translate("PageWelcome", "Form", nullptr));
        label->setText(QCoreApplication::translate("PageWelcome", "QT CART", nullptr));
        pPBStart->setText(QCoreApplication::translate("PageWelcome", "START", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PageWelcome: public Ui_PageWelcome {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEWELCOME_H
