/********************************************************************************
** Form generated from reading UI file 'pageguide.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEGUIDE_H
#define UI_PAGEGUIDE_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PageGuide
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *labelGuideTitle;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnBackToCart;
    QFrame *frame;
    QLabel *labelMap;
    QPushButton *foodIcon;
    QPushButton *groceryIcon;

    void setupUi(QWidget *PageGuide)
    {
        if (PageGuide->objectName().isEmpty())
            PageGuide->setObjectName("PageGuide");
        PageGuide->resize(532, 425);
        verticalLayout_2 = new QVBoxLayout(PageGuide);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        labelGuideTitle = new QLabel(PageGuide);
        labelGuideTitle->setObjectName("labelGuideTitle");

        horizontalLayout->addWidget(labelGuideTitle);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        btnBackToCart = new QPushButton(PageGuide);
        btnBackToCart->setObjectName("btnBackToCart");
        btnBackToCart->setCheckable(true);

        horizontalLayout->addWidget(btnBackToCart);


        verticalLayout->addLayout(horizontalLayout);

        frame = new QFrame(PageGuide);
        frame->setObjectName("frame");
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        labelMap = new QLabel(frame);
        labelMap->setObjectName("labelMap");
        labelMap->setGeometry(QRect(10, 10, 500, 300));
        labelMap->setPixmap(QPixmap(QString::fromUtf8("house.pgm")));
        labelMap->setScaledContents(true);
        foodIcon = new QPushButton(frame);
        foodIcon->setObjectName("foodIcon");
        foodIcon->setGeometry(QRect(30, 190, 50, 50));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/food.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        foodIcon->setIcon(icon);
        foodIcon->setIconSize(QSize(50, 50));
        groceryIcon = new QPushButton(frame);
        groceryIcon->setObjectName("groceryIcon");
        groceryIcon->setGeometry(QRect(430, 220, 50, 50));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/groceries.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        groceryIcon->setIcon(icon1);
        groceryIcon->setIconSize(QSize(50, 50));
        labelMap->raise();
        foodIcon->raise();
        groceryIcon->raise();

        verticalLayout->addWidget(frame);

        verticalLayout->setStretch(0, 2);
        verticalLayout->setStretch(1, 8);

        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageGuide);

        QMetaObject::connectSlotsByName(PageGuide);
    } // setupUi

    void retranslateUi(QWidget *PageGuide)
    {
        PageGuide->setWindowTitle(QCoreApplication::translate("PageGuide", "Form", nullptr));
        labelGuideTitle->setText(QCoreApplication::translate("PageGuide", "Instruction Mode", nullptr));
        btnBackToCart->setText(QCoreApplication::translate("PageGuide", "\354\236\245\353\260\224\352\265\254\353\213\210\353\241\234", nullptr));
        labelMap->setText(QString());
        foodIcon->setText(QString());
        groceryIcon->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class PageGuide: public Ui_PageGuide {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEGUIDE_H
