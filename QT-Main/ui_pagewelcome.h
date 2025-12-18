/********************************************************************************
** Form generated from reading UI file 'pagewelcome.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEWELCOME_H
#define UI_PAGEWELCOME_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
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
    QVBoxLayout *verticalLayout_3;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_3;
    QLabel *label_2;
    QLabel *label;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_5;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_6;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *pPBStart;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QWidget *PageWelcome)
    {
        if (PageWelcome->objectName().isEmpty())
            PageWelcome->setObjectName("PageWelcome");
        PageWelcome->resize(841, 593);
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
        sizePolicy.setHorizontalStretch(11);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(PageWelcome->sizePolicy().hasHeightForWidth());
        PageWelcome->setSizePolicy(sizePolicy);
        QFont font;
        font.setBold(false);
        PageWelcome->setFont(font);
        verticalLayout_2 = new QVBoxLayout(PageWelcome);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setSizeConstraint(QLayout::SizeConstraint::SetNoConstraint);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_3->setContentsMargins(2, 0, 3, -1);
        label_3 = new QLabel(PageWelcome);
        label_3->setObjectName("label_3");
        label_3->setStyleSheet(QString::fromUtf8("border-image: url(:/new/prefix1/Gemini_Generated_Image_2fvasb2fvasb2fva.png);"));

        verticalLayout_3->addWidget(label_3);


        verticalLayout->addLayout(verticalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setContentsMargins(0, -1, -1, -1);
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        label_2 = new QLabel(PageWelcome);
        label_2->setObjectName("label_2");
        label_2->setMinimumSize(QSize(79, 0));
        label_2->setStyleSheet(QString::fromUtf8("color: rgb(153, 193, 241);\n"
"font: 700 30pt \"Ria Sans\";"));

        horizontalLayout->addWidget(label_2);

        label = new QLabel(PageWelcome);
        label->setObjectName("label");
        label->setMinimumSize(QSize(0, 0));
        label->setStyleSheet(QString::fromUtf8("font: 700 25pt \"Ria Sans\";"));

        horizontalLayout->addWidget(label);

        horizontalSpacer_4 = new QSpacerItem(181, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);

        horizontalLayout->setStretch(0, 2);
        horizontalLayout->setStretch(2, 1);
        horizontalLayout->setStretch(3, 2);

        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_5);

        label_4 = new QLabel(PageWelcome);
        label_4->setObjectName("label_4");
        label_4->setMinimumSize(QSize(117, 0));
        QFont font1;
        font1.setPointSize(11);
        font1.setBold(false);
        label_4->setFont(font1);
        label_4->setLineWidth(0);

        horizontalLayout_3->addWidget(label_4);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_6);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        pPBStart = new QPushButton(PageWelcome);
        pPBStart->setObjectName("pPBStart");
        QFont font2;
        font2.setFamilies({QString::fromUtf8("Ria Sans")});
        font2.setPointSize(11);
        font2.setBold(false);
        font2.setItalic(false);
        pPBStart->setFont(font2);
        pPBStart->setStyleSheet(QString::fromUtf8("font: 11pt \"Ria Sans\";\n"
" \n"
"          background-color: #4CAF50;\n"
"          color: white;\n"
"         border-radius: 10px;\n"
"          padding: 6px 12px;\n"
"       \n"
"        hover { \n"
"         background-color: #45a049\n"
"        };\n"
"        pressed {\n"
"          background-color: #3e8e41        };"));
        pPBStart->setIconSize(QSize(250, 250));
        pPBStart->setCheckable(false);

        horizontalLayout_2->addWidget(pPBStart);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalLayout->setStretch(0, 3);

        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageWelcome);

        QMetaObject::connectSlotsByName(PageWelcome);
    } // setupUi

    void retranslateUi(QWidget *PageWelcome)
    {
        PageWelcome->setWindowTitle(QCoreApplication::translate("PageWelcome", "Form", nullptr));
        label_3->setText(QString());
        label_2->setText(QCoreApplication::translate("PageWelcome", "Qt", nullptr));
        label->setText(QCoreApplication::translate("PageWelcome", " Cart", nullptr));
        label_4->setText(QCoreApplication::translate("PageWelcome", "\354\213\234\354\236\221\355\225\230\352\270\260 \353\262\204\355\212\274\354\235\204 \353\210\214\353\237\254 \354\207\274\355\225\221\354\235\204 \354\213\234\354\236\221\355\225\230\354\204\270\354\232\224", nullptr));
        pPBStart->setText(QCoreApplication::translate("PageWelcome", "\354\213\234\354\236\221\355\225\230\352\270\260", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PageWelcome: public Ui_PageWelcome {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEWELCOME_H
