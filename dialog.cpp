#include "dialog.h"
#include "ui_dialog.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QList>
#include <QDebug>
#include <QTextEdit>
#include <QProcess>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);

    mSerialPort = new QSerialPort; //创建一个串口对象

    mIsOpen = false; //初始化按钮状态标志位

    flag_red = false;
    flag_yellow = false;
    flag_bee = false;
    //ui->btn_send->setEnabled(mIsOpen);

    //识别系统的所有可用串口号，并添加到下拉列表中
    QList<QSerialPortInfo> serialPortInfo = QSerialPortInfo::availablePorts();
    int count = serialPortInfo.count();
    for(int i = 0;i < count;i++)
    {
        ui->Cboxport->addItem(serialPortInfo.at(i).portName());
    }

    //等待一个触发信号，接收串口数据
    connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(on_SerialPort_readyRead()));


}

Dialog::~Dialog()
{
    delete ui;
}

bool Dialog::getSerialPortConfig()  //配置串口
{


    //获取串口配置
/*
    mPortName = ui->Cboxport->currentText();
    mBaudRate = ui->Cboxboudrate->currentText();
    mParity = ui->Cboxparity->currentText();
    mDataBits = ui->Cboxdatabits->currentText();
    mStopBits = ui->Cboxstopbits->currentText();
*/
    mPortName = ui->Cboxport->currentText();
    mBaudRate = "115200";
    mParity = "NONE" ;
    mDataBits = "8" ;
    mStopBits = "1";

    //设置串口
    //串口号
    mSerialPort->setPortName(mPortName);
    //波特率
    if("115200" == mBaudRate)
    {
        mSerialPort->setBaudRate(QSerialPort::Baud115200);
    }
    else
    {
        mSerialPort->setBaudRate(QSerialPort::Baud9600);
    }
    //校验位
    if("EVEN" == mParity)
    {
        mSerialPort->setParity(QSerialPort::EvenParity);
    }
    else if("ODD" == mParity)
    {
        mSerialPort->setParity(QSerialPort::OddParity);
    }
    else
    {
        mSerialPort->setParity(QSerialPort::NoParity);
    }
    //数据位
    if("5" == mDataBits)
    {
        mSerialPort->setDataBits(QSerialPort::Data5);
    }
    else if("6" == mDataBits)
    {
        mSerialPort->setDataBits(QSerialPort::Data6);
    }
    else if("7" == mDataBits)
    {
        mSerialPort->setDataBits(QSerialPort::Data7);
    }
    else
    {
        mSerialPort->setDataBits(QSerialPort::Data8);
    }
    //停止位
    if("1.5" == mStopBits)
    {
        mSerialPort->setStopBits(QSerialPort::OneAndHalfStop);
    }
    if("2" == mStopBits)
    {
        mSerialPort->setStopBits(QSerialPort::TwoStop);
    }
    else
    {
        mSerialPort->setStopBits(QSerialPort::OneStop);
    }
    qDebug() << "配置";
    return mSerialPort->open(QSerialPort::ReadWrite);

}
void Dialog::on_btn_open_clicked()  //打开关闭按钮状态
{

    if(true == mIsOpen)
    {
        //当前已经打开了串口，点击后将按钮更新为关闭状态
        mSerialPort->close();
        ui->btn_open->setText("连接设备");
        mIsOpen = false;
        //此时可以配置串口
        ui->Cboxport->setEnabled(true);
    //    ui->Cboxboudrate->setEnabled(true);
    //    ui->Cboxparity->setEnabled(true);
    //    ui->Cboxdatabits->setEnabled(true);
    //    ui->Cboxstopbits->setEnabled(true);
    //    ui->btn_send->setEnabled(mIsOpen);
        ui->textEdit_Recv-> setPlainText("");
        qDebug() << "关闭";
     //   ui->textEdit_Recv-> setPlainText("正在进行测试当中.......请稍等........");

    }
    else
    {
        //当前处于关闭串口状态，打开前需要配置串口
        //getSerialPortConfig();
        if(true == getSerialPortConfig())
        {
            mIsOpen = true;
            ui->btn_open->setText("断开设备");
            qDebug() << "成功打开串口" << mPortName;
            ui->Cboxport->setEnabled(false);
         //   ui->Cboxboudrate->setEnabled(false);
         //   ui->Cboxparity->setEnabled(false);
         //   ui->Cboxdatabits->setEnabled(false);
         //   ui->Cboxstopbits->setEnabled(false);
         //   ui->btn_send->setEnabled(mIsOpen);
            ui->textEdit_Recv-> setPlainText("设备连接成功，等待进行测试......");
        }



//                 else
//                    {
//                     mIsOpen = false;
//                    }
    }
}



/*
void Dialog::on_SerialPort_readyRead()
{


    if (true == mIsOpen)
    {
        QString recvData = mSerialPort->readAll();
       // QString text = ui->textEdit_Recv->toPlainText();
       // text += QString(recvData);
        ui->textEdit_Recv-> clear();
        ui->textEdit_Recv-> setPlainText(recvData);

        qDebug() << "正在接收数据";
    }
}
*/

int index_arr = 0;
char* dataArray = new char[8];


void Dialog::on_SerialPort_readyRead()
{

    if (true == mIsOpen)
    {
          QByteArray recvData = mSerialPort->readAll();

          int dataSize = recvData.size();


        // 逐个字节复制数据到数组中
           for (int i = 0; i < dataSize; i++) {//要从1开始，因为recvData.at(i)这个函数是从1开始的，而数组从0开始
            dataArray[index_arr] = recvData.at(i);
            qDebug() << dataArray[index_arr];

       if(QString (dataArray[0]) == "B") //功能性判断开始，设置开始服务符号为"B"，hex为42
       {
            if ( QString(dataArray[index_arr]) == "A" )//会按照总的结束符为A:41来进行设计，头一个数据作为标识符，最后一个数据作为结束符
            {
                index_arr= 0 ;//设置16进制结束符号为A,hex对应值是41.
                QPushButton* bbutton = ui->btn_bee; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_bee->setText("PASS");

            }

            if ( QString(dataArray[1]) == "P" )
            {

                QPushButton* bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_modem->setText("PASS");
            }

            if ( QString(dataArray[2]) == "P" )
            {

                QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_sim->setText("PASS");
            }

            if ( QString(dataArray[3]) == "P" )
            {

                QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_gsm->setText("PASS");
            }

            if ( QString(dataArray[4]) == "P" )
            {

                QPushButton* bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_ms->setText("PASS");
            }

            if ( QString(dataArray[5]) == "P" )
            {

                QPushButton* bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_qspi->setText("PASS");
            }

            if ( QString(dataArray[6]) == "P" )
            {

                QPushButton* bbutton = ui->btn_can; // Replace "myButton" with the object name of your QPushButton
                bbutton->setStyleSheet("background-color: green; color: white;");
                ui->btn_can->setText("PASS");
            }
       }//功能性测试判断结束

            if(QString (dataArray[0]) == "A") //电压判断开始，设置开始服务符号为"C"，hex为43
       {

            quint16 voltage_test = (dataArray[1] << 8 )| dataArray[2] ;

            float voltage_pin = voltage_test/32767.0000 * 10;
            qDebug() << "合并后的值xxxx为: " << voltage_test; // 打印合并后的值
            qDebug() << "合并后的值tttt为: " << voltage_pin; // 打印合并后的值
            ui->btn_12v_in->setText(QString::number(voltage_pin));
            QPushButton* bbutton = ui->btn_12v_in; // Replace "myButton" with the object name of your QPushButton
            if (voltage_pin >= 2)
            bbutton->setStyleSheet("background-color: green; color: white;");
            else
            bbutton->setStyleSheet("background-color: red; color: white;");

        }



            index_arr++;//正常迭代


            if (index_arr > 7)
                {

                Dialog::Serial_data_operate(dataArray, index_arr);
                index_arr = 0 ;
                }
          }


       // QString text;
        QString text = ui->textEdit_Recv->toPlainText();
        text += QString(recvData);
        ui->textEdit_Recv-> clear();
        ui->textEdit_Recv-> setPlainText(text);

        qDebug() << "正在接收数据";
        if(QString (dataArray[7]) == "S") //设置停止服务符号为"S"，hex为53
        ui->textEdit_Recv-> setPlainText("DONE");
 //       qDebug() << dataArray[0];
    }
}

void Dialog::StringToHex(QString str, QByteArray &senddata) //字符串转换为十六进制数据0-F
{
    int hexdata,lowhexdata;
    int hexdatalen = 0;
    int len = str.length();
    senddata.resize(len/2);
    char lstr,hstr;

    for(int i=0; i<len; )
    {
        //char lstr,
        hstr=str[i].toLatin1();
        if(hstr == ' ')
        {
            i++;
            continue;
        }
        i++;
        if(i >= len)
            break;
        lstr = str[i].toLatin1();
        hexdata = ConvertHexChar(hstr);
        lowhexdata = ConvertHexChar(lstr);
        if((hexdata == 16) || (lowhexdata == 16))
            break;
        else
            hexdata = hexdata*16+lowhexdata;
        i++;
        senddata[hexdatalen] = (char)hexdata;
        hexdatalen++;
    }
    senddata.resize(hexdatalen);
}

char Dialog::ConvertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
        return ch-0x30;
    else if((ch >= 'A') && (ch <= 'F'))
        return ch-'A'+10;
    else if((ch >= 'a') && (ch <= 'f'))
        return ch-'a'+10;
    else return ch-ch;//不在0-f范围内的会发送成0
}




void Dialog::on_btn_yellow_clicked()
{

    QByteArray btn_data;
    btn_data[0] = 0x01;
    btn_data[1] = 0x0d;
    btn_data[2] = 0x0a;
    mSerialPort->write(btn_data);
    if(false == flag_yellow)
     {
        ui->btn_yellow->setText("REQUESTING");
            flag_yellow = true;
     }
     else
     {
        ui->btn_yellow->setText("REQUEST");
            flag_yellow = false;
     }

}



void Dialog::on_btn_bee_clicked()
{
    QByteArray btn_data1;
    btn_data1[0] = 0x02;
    btn_data1[1] = 0x0d;
    btn_data1[2] = 0x0a;
    mSerialPort->write(btn_data1);
    if(flag_bee == false)
     {
        ui->btn_bee->setText("关闭");
            flag_bee = true;
            QPushButton* bbutton = ui->btn_bee; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
     }
    else
     {
        ui->btn_bee->setText("打开");
            flag_bee = false;
     }
}

void Dialog::on_ble_clicked()
{
  ui->textEdit_Recv-> clear();

  //这一段是JLINK 烧录的代码
  QString program = "C:/Program Files (x86)/SEGGER/JLink/JLink.exe";
  QString argument = "D:/ihex/command.txt";
 // QProcess::startDetached(program, QStringList() << argument);

  QProcess process;
  process.start(program, QStringList() << argument);
  process.waitForFinished();
/*
  QByteArray output = process.readAllStandardOutput();
  QString outputString(output);
  ui->textEdit_Recv-> append(outputString);
  //包含cmd反馈的信息显示到窗口中

  ui->textEdit_Recv-> clear();
*/ //暂时不添加jlink反馈信息
  QByteArray start_data; //for arduino to recognize to start point
  start_data[0] = 0x00;
  start_data[1] = 0x0d;
  start_data[2] = 0x0a;
  mSerialPort->write(start_data);

  ui->textEdit_Recv-> setPlainText("正在测试，等待台架返回测试结果");


}







void Dialog::on_btn_sim_clicked()
{

}


void Dialog::on_btn_gsm_clicked()
{

}


void Dialog::on_btn_ms_clicked()
{

}


void Dialog::on_btn_qspi_clicked()
{

}


void Dialog::on_btn_can_clicked()
{

}


void Dialog::on_btn_modem_clicked()
{

}



void Dialog::on_btn_12v_in_clicked()
{

}


void Dialog::on_btn_12v_outside_clicked()
{

}




void Dialog::on_btn_4V_DCDC_clicked()
{

}


void Dialog::on_btn_4V_IN_clicked()
{

}




void Dialog::on_btn_ILB_clicked()
{

}


void Dialog::on_btn_3v3_SEN_clicked()
{

}


void Dialog::on_btn_3v3_CAN_clicked()
{

}


void Dialog::on_btn_3v3_ANT_clicked()
{

}


void Dialog::on_btn_OV_clicked()
{


}

void Dialog::Serial_data_operate(char *data, int length)
{
    for (int i = 0; i < length; i++)
    {
        // 在这里对每个元素执行操作
        qDebug() << "loop里面的数值为: " << data[i];
    }
}
