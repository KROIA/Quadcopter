#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    usbDevice = new QSerialPort(this);
    connect(usbDevice,SIGNAL(readyRead()),this,SLOT(onSerialDataAvailable()));

    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(on_timerFinished()));
    //timer->start(100);
    updateSpeed = 500;

    serialDeviceIsConnected = false;
    getAvalilableSerialDevices();
    p_file = fopen("output.csv","w");
    fclose(p_file);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::getAvalilableSerialDevices()
{
    qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
    serialComPortList.clear();
    ui->serialPortSelect_comboBox->clear();
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        QString dbgStr = "Vendor ID: ";


	   if(serialPortInfo.hasVendorIdentifier())
	   {
		  dbgStr+= serialPortInfo.vendorIdentifier();
	   }
	   else
	   {
		  dbgStr+= " - ";
	   }
	   dbgStr+= "  Product ID: ";
	   if(serialPortInfo.hasProductIdentifier())
	   {
		  dbgStr+= serialPortInfo.hasProductIdentifier();
	   }
	   else
	   {
		  dbgStr+= " - ";
	   }
	   dbgStr+= " Name: " + serialPortInfo.portName();
	   dbgStr+= " Description: "+serialPortInfo.description();
	  qDebug()<<dbgStr;
	  serialComPortList.push_back(serialPortInfo);
	  ui->serialPortSelect_comboBox->addItem(serialPortInfo.portName() +" "+serialPortInfo.description());
    }
}
void MainWindow::serialWrite(QString message)
{
    if(serialDeviceIsConnected == true)
    {
        usbDevice->write(message.toUtf8()); // Send the message to the device
        qDebug() << "Message to device: "<<message;
    }
}
void MainWindow::serialRead()
{
    if(serialDeviceIsConnected == true)
    {
        serialBuffer += usbDevice->readAll(); // Read the available data
    }
}
void MainWindow::onSerialDataAvailable()
{
    if(serialDeviceIsConnected == true)
    {
        serialRead(); // Read a chunk of the Message
        //To solve that problem I send a end char "]" in My case. That helped my to know when a message is complete

        while(serialBuffer.indexOf("]") != -1) //Message complete
        {
            //serialBuffer = serialBuffer.mid(0,serialBuffer.size() -1);
            QString message = serialBuffer.mid(0,serialBuffer.indexOf("]"));
            qDebug() << "Message from device: "<<message;

            //Do something with de message here
            fileBuffer.push_back(message.toStdString());


            serialBuffer = serialBuffer.mid(serialBuffer.indexOf("]")+1);  //Clear the buffer;
        }
    }
}


void MainWindow::on_connect_button_clicked()
{
    if(serialDeviceIsConnected == false)
    {
        usbDevice->setPortName(serialComPortList[ui->serialPortSelect_comboBox->currentIndex()].portName());
        deviceDescription = serialComPortList[ui->serialPortSelect_comboBox->currentIndex()].description();
        qDebug() << "connecting to: "<<usbDevice->portName();
        if(usbDevice->open(QIODevice::ReadWrite))
        {
            //Now the serial port is open try to set configuration
            if(!usbDevice->setBaudRate(QSerialPort::Baud115200))        //Depends on your boud-rate on the Device
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setDataBits(QSerialPort::Data8))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setParity(QSerialPort::NoParity))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setStopBits(QSerialPort::OneStop))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setFlowControl(QSerialPort::NoFlowControl))
                qDebug()<<usbDevice->errorString();

            //If any error was returned the serial il corrctly configured

            qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " connected";
            serialDeviceIsConnected = true;
        }
        else
        {
            qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " not connected";
            qDebug() <<"         Error: "<<usbDevice->errorString();
            serialDeviceIsConnected = false;
        }
    }
    else
    {
        qDebug() << "Can't connect, another device is connected";
    }
}

void MainWindow::on_disconnect_button_clicked()
{
    if(serialDeviceIsConnected)
    {
        usbDevice->close();
        serialDeviceIsConnected = false;
        qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " closed";
    }
    else
    {
       qDebug() << "Can't disconnect, no device is connected";
    }
}

void MainWindow::on_pushButton_clicked()
{
   /* p_file = fopen("output.csv","a");
    if(p_file)
    {
        for(unsigned int a=0; a<fileBuffer.size(); a++)
        {
            fprintf(p_file,"%s\n",fileBuffer[a].c_str());
        }
        fclose(p_file);
        fileBuffer.clear();
    }*/
    fileBuffer.clear();
    fileIndex = ui->fileIndex_spinbox->value();
    serialWrite("lon"+QString::number(fileIndex)+"|"+QString::number(getChecksum("lon"+QString::number(fileIndex)))+"]");
    fileIndex++;
    ui->fileIndex_spinbox->setValue(fileIndex);
}
void MainWindow::on_timerFinished()
{
    QString message;
    switch(activeSlider)
    {
        case 1:
        {
            message = "p"+QString::number((int)(pow((double)ui->kp_slider->value()/(double)100,2)*100));
            break;
        }
        case 2:
        {
            message = "i"+QString::number((int)(pow((double)ui->ki_slider->value()/(double)100,2)*100));
            break;
        }
        case 3:
        {
            message = "d"+QString::number((int)(pow((double)ui->kd_slider->value()/(double)100,2)*100));
            break;
        }
        case 4:
        {
            message = "fi"+QString::number((int)(pow((double)ui->logSpeed_slider->value()/(double)100,2)*100));
            break;
        }

    }
    serialWrite(message+"|"+QString::number(getChecksum(message))+"]");
    //activeSlider = 0;
    //serialWrite("p"+QString::number(ui->kp_slider->value())+"]i"+QString::number(ui->ki_slider->value())+"]d"+QString::number(ui->kd_slider->value())+"]");
    ui->p_label->setText(QString::number(pow((double)ui->kp_slider->value()/(double)100,2)));
    ui->i_label->setText(QString::number(pow((double)ui->ki_slider->value()/(double)1000,2)));
    ui->d_label->setText(QString::number(pow((double)ui->kd_slider->value()/(double)100,2)));
}

void MainWindow::on_kp_slider_valueChanged(int value)
{
    //timer->start(100);
}

void MainWindow::on_ki_slider_valueChanged(int value)
{
    //timer->start(100);
}

void MainWindow::on_kd_slider_valueChanged(int value)
{
    //timer->start(100);
}

void MainWindow::on_kp_slider_sliderMoved(int position)
{
    //timer->start(100);
}

void MainWindow::on_kp_slider_sliderPressed()
{
    timer->start(updateSpeed);
    activeSlider = 1;
}

void MainWindow::on_kp_slider_sliderReleased()
{
    timer->stop();
}

void MainWindow::on_ki_slider_sliderPressed()
{
    timer->start(updateSpeed);
    activeSlider = 2;
}

void MainWindow::on_kd_slider_sliderPressed()
{
    timer->start(updateSpeed);
    activeSlider = 3;
}

void MainWindow::on_ki_slider_sliderReleased()
{
    timer->stop();
}

void MainWindow::on_kd_slider_sliderReleased()
{
    timer->stop();
}

void MainWindow::on_stopLog_pushButton_clicked()
{
    serialWrite("loff|"+QString::number(getChecksum("loff"))+"]");
}

void MainWindow::on_update_slider_valueChanged(int value)
{
    updateSpeed = ui->update_slider->maximum()-ui->update_slider->value();
    qDebug() << "new updateSpeed: " << updateSpeed;
}

void MainWindow::on_logSpeed_slider_valueChanged(int value)
{

}

void MainWindow::on_logSpeed_slider_sliderPressed()
{
    timer->start(updateSpeed);
    activeSlider = 4;
}

void MainWindow::on_logSpeed_slider_sliderReleased()
{
    timer->stop();
}
int MainWindow::getChecksum(QString text)
{
    int sum = 0;
    std::string _t = text.toStdString();
    for(unsigned int a=0; a<text.size(); a++)
    {
        sum += _t.at(a);
    }
    return sum;
}
