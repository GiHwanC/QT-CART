#include "mainwidget.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    MainWidget w;
    w.show();
    return a.exec();
}
