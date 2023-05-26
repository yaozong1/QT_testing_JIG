#include "dialog.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    w.setStyleSheet("background-color: #696969; color: white;");
    w.show();

    return a.exec();
}
