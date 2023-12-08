#include "dialog.h"
#include "ui_dialog.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QList>
#include <QDebug>
#include <QTextEdit>
#include <QProcess>
#include <QIcon>
#include <QTimer>

#include <QFile>
#include <QTextStream>
#include <QDate>



#define CHINESE
//#define ENGLISH



float V_dcdcboost = 0;
float V_fiveVzero = 0;
float V_AWO = 0;
float V_EBL = 0;
float V_MODEM = 0;
float V_ANT = 0;

bool vol_continue_testing = true;

bool overall_testing_result = true;


QString modem_status_result = "#";
QString motion_sensor_result = "#";
QString nrf_qspi_flash_result = "#";
QString canbus_result = "#";
QString vcu_qspi_flash_result = "#";
QString igntion_result = "#";
QString immobilizer_result = "#";

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{

    mTimer = new QTimer(this);
    mTimer->setInterval(500); // 设置超时时间为2000毫秒（2秒）
    connect(mTimer, &QTimer::timeout, this, &Dialog::onTimeout);
    //加入定时器，为UART开启超时模式，实现软件防抖动

    ui->setupUi(this);

    setWindowIcon(QIcon(":/TTC_NB.ico"));

    mSerialPort = new QSerialPort; //创建一个串口对象

    mIsOpen = false; //初始化按钮状态标志位

    flag_red = false;
    flag_yellow = false;
    flag_bee = false;
    //ui->btn_send->setEnabled(mIsOpen);
////////////////////////////////////////////////--------------设置UI的语言版本------------------------/////////////////////////////
    #ifdef ENGLISH
    ui->btn_open->setText("CONNECT");
    ui->btn_yellow->setText("Start Testing");
    ui->ble->setText("Self Test");
    ui->label_4->setText("ROAM TEST JIG V1.0");
    ui->label->setText(" TEST _ JIG _ PORT");
    ui->label_11->setText("Status/Reset(click)");

    #endif

    #ifdef CHINESE
    ui->btn_open->setText("连接");
    ui->btn_yellow->setText("开始测试");
    ui->ble->setText("模块自测");
    ui->label_4->setText("ROAM测试台架V1.0");
    ui->label->setText(" 测试端口号");
    ui->label_11->setText("状态/复位(点击)");
    // 其他中文文本...
    #endif

    ui->ble->setEnabled(false);//禁用self-test按钮
    ui->btn_yellow->setEnabled(false);//初始化之前禁用测试按键
    ui->textEdit_IMEI->setEnabled(false);//启用电压测试按键，等待工人进行下一轮测试


////////////////////////////////////////////////--------------设置UI的语言版本------------------------/////////////////////////////

/*
    //识别系统的所有可用串口号，并添加到下拉列表中
    QList<QSerialPortInfo> serialPortInfo = QSerialPortInfo::availablePorts();
    int count = serialPortInfo.count();
    for(int i = 0;i < count;i++)
    {
        //ui->Cboxport->addItem(serialPortInfo.at(i).portName());//老方法没有添加描述的



        QString portName = serialPortInfo.at(i).portName();
        QString description = serialPortInfo.at(i).description();
        ui->Cboxport->addItem(QString("%1(%2)").arg(portName, description));


    }

 */


    ui->Cboxport->addItem("COM30"); //上面注释的是获取端口号的，这里不获取了默认就为COM9


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
    //mPortName = ui->Cboxport->currentText();     //一开始用的办法

    QString fullText = ui->Cboxport->currentText();  // 获取 "COM7(JLINK)"
    QStringList parts = fullText.split('(');  // 使用 '(' 分割字符串

    if(!parts.isEmpty()) {
        mPortName = parts.first();  // 取分割后的第一部分，即 "COM7"
    }



    mBaudRate = "9600";
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

#ifdef ENGLISH
       ui->btn_open->setText("CONNECT");

#endif

#ifdef CHINESE
        ui->btn_open->setText("连接");
        // 其他中文文本...
#endif

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

#ifdef ENGLISH
            ui->btn_open->setText("DISCONNECT");

#endif

#ifdef CHINESE
            ui->btn_open->setText("断开连接");

#endif

            qDebug() << "成功打开串口" << mPortName;
            ui->Cboxport->setEnabled(false);
         //   ui->Cboxboudrate->setEnabled(false);
         //   ui->Cboxparity->setEnabled(false);
         //   ui->Cboxdatabits->setEnabled(false);
         //   ui->Cboxstopbits->setEnabled(false);
         //   ui->btn_send->setEnabled(mIsOpen);
#ifdef ENGLISH
            ui->textEdit_Recv-> setPlainText("DUT connected sucessfully, waiting for being tested......");

            QPushButton* bbutton = ui->btn_bee; //提示需要复位之后才可以开始测试
            bbutton->setStyleSheet("background-color: #403F3C; color: white;");
            ui->btn_bee->setText("Please click to init...");
#endif

#ifdef CHINESE
            ui->textEdit_Recv-> setPlainText("设备连接成功，等待测试");
            QPushButton* bbutton = ui->btn_bee;
            bbutton->setStyleSheet("background-color: #09FFC0; color: black;");
            ui->btn_bee->setText("请单击，以初始化台架");

#endif




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

unsigned char* dataArray = new unsigned char[20];//改成unsigned char就没问题了,之前是char导致后续运算溢出，比如接受超过最大值一半的数，就会溢出

void Dialog::onTimeout()
{
    // 超时发生时的处理
    memset(dataArray, 0, 20); // 清空dataArray
    index_arr = 0; // 重置index_arr
    qDebug() << "超时，清空uart数据";

    mTimer->stop(); // 停止定时器
}

void Dialog::on_SerialPort_readyRead()
{

    if (true == mIsOpen)
    {

          QByteArray recvData = mSerialPort->readAll();

          int dataSize = recvData.size();



        // 逐个字节复制数据到数组中
           for (int i = 0; i < dataSize; i++)//要从1开始，因为recvData.at(i)这个函数是从1开始的，而数组从0开始
          {

            if (index_arr == 0) {
                qDebug() << "UART接收开始";
                mTimer->start(); // 当开始接收数据时，启动定时器
            }

            dataArray[index_arr] = recvData.at(i);
            qDebug() << dataArray[index_arr];

            index_arr++;//正常迭代

            if (index_arr > 19)
                {

                Dialog::Serial_data_operate(dataArray, index_arr);
                memset(dataArray, 0, 20);
                qDebug() << "处理完，清空uart数据";
                index_arr = 0 ;
                }
            mTimer->start(); // 每次接收到新数据时重置定时器

          }

       // QString text;

          //以下是在窗口显示串口接收数据的
/*
        QString text = ui->textEdit_Recv->toPlainText();
        text += QString(recvData);
        ui->textEdit_Recv-> clear();
        ui->textEdit_Recv-> setPlainText(text);
*/
        qDebug() << "Receiving datas....";
     //   ui->textEdit_Recv-> setPlainText("DONE");

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

        overall_testing_result = true; //每次测试之前复位总测试结果
        vol_continue_testing = true;
        //这一段是JLINK 烧录的代码

        QString program = "JLink.exe";
        QString argument = "../ihex/command_vol.txt";
        //QProcess::startDetached(program, QStringList() << argument);

        QProcess process;
        process.start(program, QStringList() << argument);
        process.waitForFinished();

      //  system("JLink.exe  ../ihex/command_vol.txt");

        QByteArray btn_data;
        btn_data.resize(3);
        btn_data[0] = 0x01;
        btn_data[1] = 0x0d;
        btn_data[2] = 0x0a;
        mSerialPort->write(btn_data);


        QPushButton* bbutton = ui->btn_bee; // 清空数据右下角按键提示数据
        bbutton->setStyleSheet("");
        bbutton->setText("");

        ui->progressBar->setValue(10);//进度条

        ui->btn_yellow->setEnabled(false);//禁用电压测试按键，避免多次点击导致错误


}




void Dialog::on_ble_clicked()
{
  ui->textEdit_Recv-> clear();

  //这一段是JLINK 烧录的代码
  QString program = "JLink.exe";
  QString argument = "../ihex/fw_loading.txt";
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
  start_data.resize(3);
  start_data[0] = 0x00;
  start_data[1] = 0x0d;
  start_data[2] = 0x0a;
  mSerialPort->write(start_data);

  index_arr = 0;
  qDebug() << "清空index，等待下一次数据";

#ifdef ENGLISH
  ui->textEdit_Recv-> setPlainText("Testing Fw loading done");

#endif

#ifdef CHINESE
  ui->textEdit_Recv-> setPlainText("测试程序写入成功");

#endif

  //////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
  ui->progressBar->setValue(50);//进度条
  // 强制处理所有挂起的事件，确保UI更新
  QApplication::processEvents();
  ////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

}






void Dialog::Serial_data_operate(unsigned char *data, int length)//很重要的点，QTserial是按照字符的格式传过来的，每个数据接收后默认就是字符，如果要计算，要转换之后才能计算，不然容易出错
{
    qDebug() << "uart循环进入";
    if(QString (data[0]) == "A" && QString (data[length-1]) == "S")
  {
//for 4V7_Boost
        qDebug() << "处理电压测试数据";
        quint16 voltage_test = 0;
        float voltage_pin = 0   ;

        voltage_test = (data[1] << 8 )| data[2] ;
        //quint16 voltage_test = (static_cast<quint16>(data[1]) << 8) | static_cast<quint8>(data[2]);
        //这个是GPT给出的格式，强制吧data[1]和[2]变成uint之后踩进行移位和其他运算,static_cast<quint16>是C++格式版本的强制类型转换，相对保守和安全
        //不过把指针类型改成无符号之后就不需要后续进行转换了，ASCII对应的值都是无符号，所以后续直接==“A”这种操作没有问题
        voltage_pin = voltage_test/32767.00 * 10;
        V_dcdcboost =voltage_pin; //for saving to csv,sometimes we will change the sequence,so add one more step.

        ui->btn_4V7_BOOST->setText(QString::number(voltage_pin));
        QPushButton* bbutton = ui->btn_4V7_BOOST; // Replace "myButton" with the object name of your QPushButton
        if (voltage_pin >= 4.559 && voltage_pin <= 4.841) // 4.7V+-3%
        bbutton->setStyleSheet("background-color: green; color: white;");
        else

           {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
          }

//For 5V0
         voltage_test = (data[3] << 8 )| data[4] ;
         //voltage_test = (static_cast<quint16>(data[3]) << 8) | static_cast<quint8>(data[4]);
         voltage_pin = voltage_test/32767.00 * 10;
         V_fiveVzero = voltage_pin; //for saving to csv
        // qDebug() << data[3];
         ui->btn_V5V0->setText(QString::number(voltage_pin));
         bbutton = ui->btn_V5V0; // Replace "myButton" with the object name of your QPushButton
         if (voltage_pin >= 4.85 && voltage_pin <= 5.15)// 5V+-3%
        bbutton->setStyleSheet("background-color: green; color: white;");
        else

         {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
          }

//For 3V3_ALWAYS
        voltage_test = (data[5] << 8 )| data[6] ;
        //voltage_test = (static_cast<quint16>(data[5]) << 8) | static_cast<quint8>(data[6]);
        voltage_pin = voltage_test/32767.00 * 10;
        V_AWO = voltage_pin; //for saving to csv
        // qDebug() << data[3];
        ui->btn_V3V3AWO->setText(QString::number(voltage_pin));
        bbutton = ui->btn_V3V3AWO; // Replace "myButton" with the object name of your QPushButton
        if (voltage_pin >= 3.201 && voltage_pin <= 3.399)// 3.3V+-3%
        bbutton->setStyleSheet("background-color: green; color: white;");
        else
         {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
          }

//For EBL
        voltage_test = (data[7] << 8 )| data[8] ;
       // voltage_test = (static_cast<quint16>(data[7]) << 8) | static_cast<quint8>(data[8]);
        voltage_pin = voltage_test/32767.00 * 10;
        V_EBL = voltage_pin; //for saving to csv

        // qDebug() << data[3];
        ui->btn_EBL->setText(QString::number(voltage_pin));
        bbutton = ui->btn_EBL; // Replace "myButton" with the object name of your QPushButton
        if (voltage_pin >= 0.3677 && voltage_pin <= 0.4064)// 0.3870V+-5%
          bbutton->setStyleSheet("background-color: green; color: white;");
        else

          {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
          }

//For 4V0_MODEM
         voltage_test = (data[9] << 8 )| data[10] ;
        // voltage_test = (static_cast<quint16>(data[9]) << 8) | static_cast<quint8>(data[10]);
         voltage_pin = voltage_test/32767.00 * 10;
         V_MODEM = voltage_pin;
        // qDebug() << data[3];
         ui->btn_4V0_MODEM->setText(QString::number(voltage_pin));
         bbutton = ui->btn_4V0_MODEM; // Replace "myButton" with the object name of your QPushButton
         if (voltage_pin >= 3.88 && voltage_pin <= 4.12) //4v+-3%
            bbutton->setStyleSheet("background-color: green; color: white;");
         else

         {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
         }

//For 3v3_CAN
         voltage_test = (data[11] << 8 )| data[12] ;
        // voltage_test = (static_cast<quint16>(data[11]) << 8) | static_cast<quint8>(data[12]);
         voltage_pin = voltage_test/32767.00 * 10;
         V_ANT = voltage_pin;
         // qDebug() << data[3];
         ui->btn_3v3_ANT->setText(QString::number(voltage_pin));
         bbutton = ui->btn_3v3_ANT; // Replace "myButton" with the object name of your QPushButton
         if (voltage_pin >= 3.201 && voltage_pin <= 3.399)// 3.3V+-3%
         bbutton->setStyleSheet("background-color: green; color: white;");
         else
         {
           vol_continue_testing = false;
           bbutton->setStyleSheet("background-color: red; color: white;");
         }

        ui->progressBar->setValue(30);//进度条

         // 强制处理所有挂起的事件，确保UI更新
         QApplication::processEvents();


         /////////////////////////////////------------- //如果电压测试成功改了，就自动进行下一步------------/////////////////////////////

         if (vol_continue_testing == true)
         {
           on_ble_clicked();
           qDebug() << "电压测试通过，进入下一轮测试";
             #ifdef ENGLISH
              ui->textEdit_Recv-> setPlainText("Voltage testing pass, entered self testing, waiting for result");

             #endif

             #ifdef CHINESE
              ui->textEdit_Recv-> setPlainText("电压测试通过，进入自测模式，等待测试结果");

             #endif

          // ui->textEdit_Recv-> setPlainText("1234567");


         }

         if (vol_continue_testing == false)
         {
           //on_ble_clicked();
           qDebug() << "电压测试失败";


           ui->btn_yellow->setEnabled(false);//电压测试失败之后禁用测试按键，提示初始化

           #ifdef ENGLISH

               ui->textEdit_Recv-> setPlainText("Voltage testing failed， please click & test again or swap device");
               QPushButton* bbutton = ui->btn_bee; //提示需要复位之后才可以开始测试
               ui->btn_bee->setText("Error, please click to init...");
               bbutton->setStyleSheet("background-color: #403F3C; color: white;");


           #endif

           #ifdef CHINESE

               ui->textEdit_Recv-> setPlainText("电压测试失败，请点击右下角按键后再次测试，或更换测试设备");
               QPushButton* bbutton = ui->btn_bee; //提示需要复位之后才可以开始测试
               bbutton->setStyleSheet("background-color: #403F3C; color: white;");
               ui->btn_bee->setText("失败，请单击，以初始化台架");
           #endif



           ui->progressBar->setValue(100);//进度条

         }



         /////////////////////////////////------------- //如果电压测试成功改了，就自动进行下一步------------////////////////////////////


    }





    if(QString (data[0]) == "B" && QString (data[length-1]) == "S") //功能性判断开始，设置开始服务符号为"B"，hex为42,停止位为"S".
    {
/////////
          qDebug() << "处理自测试结果数据";
         if ( QString(data[1]) == "P" )
         {

             QPushButton* bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_modem->setText("PASS");
             modem_status_result = "PASS";
         }

         else
         {

              QPushButton* bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_modem->setText("FAIL");
              modem_status_result = "FAIL";
              overall_testing_result = false;
         }

/////////////

         if ( QString(data[2]) == "P" )
         {

             QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_sim->setText("PASS");


         }
         else if ( QString(data[2]) == "K" )//SKIP THE GSM TESTING
         {

            // QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
            // bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_sim->setText("SKIP");

         }
         else
         {

              QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_sim->setText("FAIL");
              overall_testing_result = false;
         }

////////////
         if ( QString(data[3]) == "P" )
         {

             QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_gsm->setText("PASS");
         }
         else if ( QString(data[3]) == "K" )//SKIP THE GSM TESTING
         {

            // QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
            // bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_gsm->setText("SKIP");
         }
         else
         {

              QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_gsm->setText("FAIL");
              overall_testing_result = false;
         }

////////////



         if ( QString(data[4]) == "P" )
         {

             QPushButton* bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_ms->setText("PASS");
             motion_sensor_result = "PASS";
         }
         else
         {

              QPushButton* bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_ms->setText("FAIL");
              motion_sensor_result = "FAIL";
              overall_testing_result = false;
         }
///////////
///
///
///
///
         if ( QString(data[5]) == "P" )
         {

             QPushButton* bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_qspi->setText("PASS");
             nrf_qspi_flash_result = "PASS";
         }

         else
         {

              QPushButton* bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_qspi->setText("FAIL");
              nrf_qspi_flash_result = "FAIL";
              overall_testing_result = false;
         }
///////////////

         if ( QString(data[6]) == "P" )
         {

             QPushButton* bbutton = ui->btn_can; // Replace "myButton" with the object name of your QPushButton
             bbutton->setStyleSheet("background-color: green; color: white;");
             ui->btn_can->setText("PASS");
             canbus_result = "PASS";
         }
         else
         {

              QPushButton* bbutton = ui->btn_can; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_can->setText("FAIL");
              canbus_result = "FAIL";
              overall_testing_result = false;
         }

         ///////////////

         if ( QString(data[7]) == "P" )
         {

              QPushButton* bbutton = ui->btn_vcu_flash; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: green; color: white;");
              ui->btn_vcu_flash->setText("PASS");
              vcu_qspi_flash_result = "PASS";
         }
         else
         {

              QPushButton* bbutton = ui->btn_vcu_flash; // Replace "myButton" with the object name of your QPushButton
              bbutton->setStyleSheet("background-color: red; color: white;");
              ui->btn_vcu_flash->setText("FAIL");
              vcu_qspi_flash_result = "FAIL";
              overall_testing_result = false;
         }



//////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
            ui->progressBar->setValue(70);//进度条
            // 强制处理所有挂起的事件，确保UI更新
            QApplication::processEvents();
////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

    }//功能性测试判断结束


    if(QString (data[0]) == "C" && QString (data[length-1]) == "S") //状态判断开始，设置开始服务符号为"B"，hex为42,停止位为"S".
    {
        if ( QString(data[1]) == "W" )
        {

            on_btn_bee_clicked();//jig提起就自动按下这个案件，重置ESP32和清除面板数据
            QPushButton* bbutton = ui->btn_bee; //显示是否提起了JIG



           #ifdef ENGLISH

               bbutton->setStyleSheet("background-color: #09FFC0; color: black;");
               ui->btn_bee->setText("Detached, please swap device");

           #endif

           #ifdef CHINESE

           bbutton->setStyleSheet("background-color: #09FFC0; color: black;");
           ui->btn_bee->setText("已释放，请更换板子");

           #endif


           //////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
           ui->progressBar->setValue(0);//进度条
           // 强制处理所有挂起的事件，确保UI更新
           QApplication::processEvents();
           ////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

        }

    }//状态报告判断结束


    if(QString (data[0]) == "C" && QString (data[length-1]) == "S") //状态判断开始，设置开始服务符号为"B"，hex为42,停止位为"S".
    {
        if ( QString(data[1]) == "E" )
        {

            QPushButton* bbutton = ui->btn_can; //显示是否提起了JIG
            bbutton->setStyleSheet("background-color: red; color: white;");
            #ifdef ENGLISH
            ui->btn_can->setText("CANBUS TIMEOUT");

            #endif

            #ifdef CHINESE
                  ui->btn_can->setText("CANBUS等待超时");

            #endif





            ui->textEdit_Recv-> clear();
            ui->textEdit_Recv-> setPlainText("Testing Fw loading done");

            bbutton = ui->btn_bee; //显示是否提起了JIG
            bbutton->setStyleSheet("background-color: red; color: white;");

             #ifdef ENGLISH
              ui->btn_bee->setText("Timeout...");

             #endif

             #ifdef CHINESE
                ui->btn_bee->setText("超时...");

             #endif

//////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
                ui->progressBar->setValue(60);//进度条
                // 强制处理所有挂起的事件，确保UI更新
                QApplication::processEvents();
////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////
        }

    }//状态报告判断结束



    if(QString (data[0]) == "c" && QString (data[10]) == "q") //这个是CAN失败，才会触发的代码//这里很特殊，是第一位和第十一位做判断
    {

        qDebug() << "处理自测试结果数据,来自ESP32 UART";
        if ( QString(data[1]) == "P" )
        {

            QPushButton* bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_modem->setText("PASS");
            modem_status_result = "PASS";
        }

        else
        {

            QPushButton* bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_modem->setText("FAIL");
            modem_status_result = "FAIL";
        }

        /////////////

        if ( QString(data[2]) == "P" )
        {

            QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_sim->setText("PASS");


        }
        else if ( QString(data[2]) == "K" )//SKIP THE GSM TESTING
        {

            // QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
            // bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_sim->setText("SKIP");

        }
        else
        {

            QPushButton* bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_sim->setText("FAIL");
        }

        ////////////
        if ( QString(data[3]) == "P" )
        {

            QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_gsm->setText("PASS");
        }
        else if ( QString(data[3]) == "K" )//SKIP THE GSM TESTING
        {

            // QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
            // bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_gsm->setText("SKIP");
        }
        else
        {

            QPushButton* bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_gsm->setText("FAIL");
        }

        ////////////



        if ( QString(data[4]) == "P" )
        {

            QPushButton* bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_ms->setText("PASS");
            motion_sensor_result = "PASS";
        }
        else
        {

            QPushButton* bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_ms->setText("FAIL");
            motion_sensor_result = "FAIL";
        }
        ///////////
        ///
        ///
        ///
        ///
        if ( QString(data[5]) == "P" )
        {

            QPushButton* bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_qspi->setText("PASS");
            nrf_qspi_flash_result = "PASS";
        }

        else
        {

            QPushButton* bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_qspi->setText("FAIL");
            nrf_qspi_flash_result = "FAIL";
        }
        ///////////////

 //直接给判断，忽略数组里面can的默认result，因为没收到，所以就判断为错误



            QPushButton* bbutton = ui->btn_can; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_can->setText("FAIL");
            canbus_result = "FAIL";
            overall_testing_result = false;

/////////////////////////////
///
///
        ///////////////

        if ( QString(data[7]) == "P" )
        {

            QPushButton* bbutton = ui->btn_vcu_flash; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_vcu_flash->setText("PASS");
            vcu_qspi_flash_result = "PASS";
        }
        else
        {

            QPushButton* bbutton = ui->btn_vcu_flash; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_vcu_flash->setText("FAIL");
            vcu_qspi_flash_result = "FAIL";
            overall_testing_result = false;
        }


 //////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
        ui->progressBar->setValue(70);//进度条
        // 强制处理所有挂起的事件，确保UI更新
        QApplication::processEvents();
 ////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

    }//状态报告判断结束

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   if(QString (data[0]) == "q" && QString (data[length-1]) == "s")
    {
        qDebug() << "收到IMEI";
        // QString text;
        QString text = ui->textEdit_IMEI->toPlainText();


        if ( QString(data[17]) == "P" )//它收到的是27-10，第17位
        {

            QPushButton* bbutton = ui->btn_vcu_ign; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_vcu_ign->setText("PASS");
            igntion_result = "PASS";
        }
        else
        {

            QPushButton* bbutton = ui->btn_vcu_ign; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_vcu_ign->setText("FAIL");
            igntion_result = "FAIL";
            overall_testing_result = false;
        }


        if ( QString(data[18]) == "P" )//它收到的是27-10，第17位
        {

            QPushButton* bbutton = ui->btn_vcu_imout; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_vcu_imout->setText("PASS");
            immobilizer_result = "PASS";

        }
        else
        {

            QPushButton* bbutton = ui->btn_vcu_imout; // Replace "myButton" with the object name of your QPushButton
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_vcu_imout->setText("FAIL");
            immobilizer_result = "FAIL";
            overall_testing_result = false;
        }


//////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
        ui->progressBar->setValue(90);//进度条
        // 强制处理所有挂起的事件，确保UI更新
        QApplication::processEvents();
////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

        for(int i = 1; i<16; i++)
        {
           // qDebug() << "Value of i:" << i;
            text += QString(data[i]);
        }

        ui->textEdit_IMEI-> clear();
        ui->textEdit_IMEI->setPlainText(text);

        QString imei = text; // 从UI获取IMEI


        saveToCsv(imei);

       // QString fullText = "IMEI: " + text;
       // ui->textEdit_IMEI->setPlainText(fullText);



      /////////////////////////////////////////////////////////////////////////////////////////////////////////

        //QPushButton* bbutton = ui->btn_bee; //显示是否打开了JIG,测试完就是提示请detach jig
        //bbutton->setStyleSheet("background-color: green; color: white;");


        #ifdef ENGLISH
        if(overall_testing_result == true)

        {
            QPushButton* bbutton = ui->btn_bee; //显示是否提起了JIG
            bbutton->setStyleSheet("background-color: green; color: white;");
            ui->btn_bee->setText("Tested successfully, please detach the device...");
        }
        else
        {
            QPushButton* bbutton = ui->btn_bee; //显示是否提起了JIG
            bbutton->setStyleSheet("background-color: red; color: white;");
            ui->btn_bee->setText("Tested failed, please detach the device..");
        }

        #endif

        #ifdef CHINESE
        if(overall_testing_result == true)
        {
        QPushButton* bbutton = ui->btn_bee; //显示是否提起了JIG
        bbutton->setStyleSheet("background-color: green; color: white;");
        ui->btn_bee->setText("测试成功，请释放板子，并取出");
        }


        else
        {
        QPushButton* bbutton = ui->btn_bee; //显示是否提起了JIG
        bbutton->setStyleSheet("background-color: red; color: white;");
        ui->btn_bee->setText("测试不通过，请释放板子，并取出");
        }

        #endif



        ui->textEdit_Recv-> clear();


        #ifdef ENGLISH
        ui->textEdit_Recv-> setPlainText("Testing Done");

        #endif

        #ifdef CHINESE
        ui->textEdit_Recv-> setPlainText("测试已完成");

        #endif

 //////////////////////////////////////////////////////// ---------更新进度条&UI-----------/////////////////////////////////
        ui->progressBar->setValue(100);//进度条
        // 强制处理所有挂起的事件，确保UI更新
        QApplication::processEvents();
 ////////////////////////////////////////////////////////////---------更新进度条&UI-----------/////////////////////////////////

        //detach前要给板子清空数据


        //这一段是JLINK 烧录的代码
        // QString program = "C:/Program Files (x86)/SEGGER/JLink/JLink.exe";
        // QString argument = "../ihex/command_erase.txt";
        // QProcess::startDetached(program, QStringList() << argument);

        // QProcess process;
        // process.start(program, QStringList() << argument);
        // process.waitForFinished();

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void Dialog::saveToCsv(const QString& imei)//void Dialog::saveToCsv(const QString& imei, const QString& volTest, const QString& selfTest)
{
    QFile file("../test_results.csv");

    // 打开文件用于追加
    if (!file.open(QIODevice::Append | QIODevice::Text))
        return;

    QTextStream out(&file);

    // 检查文件是否为空（即是否首次写入）
    if (file.size() == 0) {
        // 写入列标题
        out << ",,VOLTAGE PROBING,,,,,FUNCTION TESTING,,,,,,\n";
        out << "Date,IMEI_number,4V7_BOOST,5V0,3V3_AWO,EBL,4V0_MODEM,3V3_ANT,SIM_MODEM,MOTION_SENSOR,NRF_FLASH,CAN_BUS,VCU_FLASH,IGNITION,IMMOBILIZER,OVERALL_RESULT\n";
    }


    QString overall_testing_result_str = "#";
    if(overall_testing_result == true)
        overall_testing_result_str = "PASS";
    else
        overall_testing_result_str = "FAIL";

    /*
    float V_dcdcboost = 0;
    float V_fiveVzero = 0;
    float V_AWO = 0;
    float V_EBL = 0;
    float V_MODEM = 0;
    float V_ANT = 0;


    QString modem_status_result = "#";
    QString motion_sensor_result = "#";
    QString nrf_qspi_flash_result = "#";
    QString canbus_result = "#";
    QString vcu_qspi_flash_result = "#";
    QString igntion_result = "#";
    QString immobilizer_result = "#";
*/
    // 获取当前日期
    QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm");
    qDebug() << "Current DateTime:" << currentDateTime;  // 调试输出，检查格式化后的字符串

    // 写入数据

    out << currentDateTime  << "," << imei << "," << V_dcdcboost << "," << V_fiveVzero << ","<< V_AWO << ","<< V_EBL << ","<< V_MODEM<< ","<< V_ANT << ","<< modem_status_result << "," << motion_sensor_result<< ","<< nrf_qspi_flash_result << "," << canbus_result<< ","<< vcu_qspi_flash_result << ","<< igntion_result<< "," << immobilizer_result<< "," << overall_testing_result_str << "\n";

    file.close();
}


void Dialog::on_btn_bee_clicked()
{
    index_arr = 0;
    qDebug() << "清空index，等待下一次数据";

    qDebug() << "Reset ESP32";
    QByteArray btn_data_begin;
    btn_data_begin.resize(3);
    btn_data_begin[0] = 0x09;
    btn_data_begin[1] = 0x09;
    btn_data_begin[2] = 0x06;
    mSerialPort->write(btn_data_begin);


    QPushButton* bbutton = ui->btn_4V7_BOOST; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_V5V0; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_V3V3AWO; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");


    bbutton = ui->btn_EBL; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");


    bbutton = ui->btn_4V0_MODEM; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_3v3_ANT; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_bee; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_modem; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_sim; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_gsm; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");


    bbutton = ui->btn_ms; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_qspi; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_can; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_vcu_flash; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_vcu_imout; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    bbutton = ui->btn_vcu_ign; // Replace "myButton" with the object name of your QPushButton
    bbutton->setStyleSheet("");
    bbutton->setText("");

    ui->textEdit_Recv-> setPlainText("");//清除文字
    ui->textEdit_IMEI-> clear();//清除QT文字

    ui->progressBar->setValue(0);//进度条

    ui->btn_yellow->setEnabled(true);//启用电压测试按键，等待工人进行下一轮测试

}










void Dialog::on_btn_EBL_clicked()
{

}

