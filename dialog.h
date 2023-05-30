#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QSerialPort>

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(QWidget *parent = nullptr);
    ~Dialog();

public:
    bool getSerialPortConfig(void);




private slots:  //槽函数，遇到相应信号就触发
    void on_btn_open_clicked();

    void on_btn_send_clicked();

    void on_SerialPort_readyRead();

    void StringToHex(QString str, QByteArray &senddata);
    char ConvertHexChar(char ch);

    void on_btn_yellow_clicked();

    void on_btn_red_clicked();

    void on_btn_bee_clicked();

    void on_ble_clicked();


    void on_textEdit_Recv_copyAvailable(bool b);

    void on_textEdit_Recv_textChanged();

private:
    Ui::Dialog *ui;

    bool mIsOpen;  //串口按钮是否打开
     //串口对象指针及配置参数
    QSerialPort *mSerialPort;
    QString mPortName;
    QString mBaudRate;
    QString mParity;
    QString mDataBits;
    QString mStopBits;

    bool flag_yellow;
    bool flag_red;
    bool flag_bee;
    bool flag_ble;


};
#endif // DIALOG_H
