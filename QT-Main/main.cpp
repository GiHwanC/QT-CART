#include "mainwidget.h"
#include <QApplication>
#include <QScreen>

int main(int argc, char *argv[])
{
    // (선택) DPI 흔들림 줄이기 — QApplication 생성 전에
    QCoreApplication::setAttribute(Qt::AA_Use96Dpi);

    QApplication a(argc, argv);

    // (선택) 스타일/폰트 통일
    QApplication::setStyle("Fusion");
    // QApplication::setFont(QFont("Noto Sans CJK KR", 10));

    MainWidget w;

    // ✅ 800x480 고정
    w.setFixedSize(800, 480);

    // ✅ 타이틀바까지 꽉 채우고 싶으면 FullScreen
    w.showFullScreen();

    // (대안) 창모드로 크게만: w.show();
    // (대안) 최대화: w.showMaximized();

    return a.exec();
}
