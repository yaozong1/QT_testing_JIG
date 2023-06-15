
#include "dialog.h"

#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    w.setStyleSheet("background-color: #393A3F; color: white;");
    //w.setWindowFlags(Qt::FramelessWindowHint); // 设置窗口为无边框模式
    w.show();

    return a.exec();

}
