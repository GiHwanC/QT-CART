QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

SOURCES += \
    barcodescanner.cpp \
    main.cpp \
    mainwidget.cpp \
    pagecart.cpp \
    pageguide.cpp \
    pagepay.cpp \
    pagepay_card.cpp \
    pagetotalpay.cpp \
    pagewelcome.cpp

HEADERS += \
    barcodescanner.h \
    item.h \
    mainwidget.h \
    pagecart.h \
    pageguide.h \
    pagepay.h \
    pagepay_card.h \
    pagetotalpay.h \
    pagewelcome.h

FORMS += \
    mainwidget.ui \
    pagecart.ui \
    pageguide.ui \
    pagepay.ui \
    pagepay_card.ui \
    pagetotalpay.ui \
    pagewelcome.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES +=

RESOURCES += \
    etc_image/etc_image.qrc \
    item_image/item_image.qrc
