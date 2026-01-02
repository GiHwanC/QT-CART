#include "mainwidget.h"
#include <QApplication>
#include <QShortcut>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWidget w;

    w.setFixedSize(800,  480);

    // 1) 시작하자마자 풀스크린
    w.showFullScreen();

    // 2) F11로 풀스크린 토글
    auto *sc = new QShortcut(QKeySequence(Qt::Key_F11), &w);
    sc->setContext(Qt::ApplicationShortcut); // 포커스가 다른 위젯에 있어도 동작
    QObject::connect(sc, &QShortcut::activated, [&w]() {
        if (w.isFullScreen()) w.showNormal();   // 또는 showMaximized() 원하면 그걸로
        else w.showFullScreen();
    });

    return a.exec();
}
