/****************************************************************************
** Meta object code from reading C++ file 'pagecart.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "pagecart.h"
#include <QtGui/qtextcursor.h>
#include <QtNetwork/QSslError>
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pagecart.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN8PageCartE_t {};
} // unnamed namespace


#ifdef QT_MOC_HAS_STRINGDATA
static constexpr auto qt_meta_stringdata_ZN8PageCartE = QtMocHelpers::stringData(
    "PageCart",
    "guideModeClicked",
    "",
    "goWelcome",
    "goPay",
    "onPlusClicked",
    "onMinusClicked",
    "onDeleteClicked",
    "onBarcodeEntered",
    "handleItemFetched",
    "Item",
    "item",
    "handleFetchFailed",
    "err",
    "on_btnGuideMode_clicked",
    "on_pushButton_clicked",
    "on_btnPay_clicked",
    "processPendingDatagrams"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA

Q_CONSTINIT static const uint qt_meta_data_ZN8PageCartE[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   92,    2, 0x06,    1 /* Public */,
       3,    0,   93,    2, 0x06,    2 /* Public */,
       4,    0,   94,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       5,    0,   95,    2, 0x08,    4 /* Private */,
       6,    0,   96,    2, 0x08,    5 /* Private */,
       7,    0,   97,    2, 0x08,    6 /* Private */,
       8,    0,   98,    2, 0x08,    7 /* Private */,
       9,    1,   99,    2, 0x08,    8 /* Private */,
      12,    1,  102,    2, 0x08,   10 /* Private */,
      14,    0,  105,    2, 0x08,   12 /* Private */,
      15,    0,  106,    2, 0x08,   13 /* Private */,
      16,    0,  107,    2, 0x08,   14 /* Private */,
      17,    0,  108,    2, 0x08,   15 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject PageCart::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ZN8PageCartE.offsetsAndSizes,
    qt_meta_data_ZN8PageCartE,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_tag_ZN8PageCartE_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<PageCart, std::true_type>,
        // method 'guideModeClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'goWelcome'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'goPay'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onPlusClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onMinusClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onDeleteClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onBarcodeEntered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleItemFetched'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const Item &, std::false_type>,
        // method 'handleFetchFailed'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'on_btnGuideMode_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_pushButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_btnPay_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'processPendingDatagrams'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void PageCart::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<PageCart *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->guideModeClicked(); break;
        case 1: _t->goWelcome(); break;
        case 2: _t->goPay(); break;
        case 3: _t->onPlusClicked(); break;
        case 4: _t->onMinusClicked(); break;
        case 5: _t->onDeleteClicked(); break;
        case 6: _t->onBarcodeEntered(); break;
        case 7: _t->handleItemFetched((*reinterpret_cast< std::add_pointer_t<Item>>(_a[1]))); break;
        case 8: _t->handleFetchFailed((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 9: _t->on_btnGuideMode_clicked(); break;
        case 10: _t->on_pushButton_clicked(); break;
        case 11: _t->on_btnPay_clicked(); break;
        case 12: _t->processPendingDatagrams(); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _q_method_type = void (PageCart::*)();
            if (_q_method_type _q_method = &PageCart::guideModeClicked; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _q_method_type = void (PageCart::*)();
            if (_q_method_type _q_method = &PageCart::goWelcome; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _q_method_type = void (PageCart::*)();
            if (_q_method_type _q_method = &PageCart::goPay; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject *PageCart::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PageCart::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ZN8PageCartE.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int PageCart::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void PageCart::guideModeClicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void PageCart::goWelcome()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void PageCart::goPay()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
