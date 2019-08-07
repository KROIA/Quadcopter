#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <vector>


namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();


    private slots:
        void onSerialDataAvailable();
        void on_connect_button_clicked();
        void on_disconnect_button_clicked();

        void on_pushButton_clicked();
        void on_timerFinished();

        void on_kp_slider_valueChanged(int value);

        void on_ki_slider_valueChanged(int value);

        void on_kd_slider_valueChanged(int value);

        void on_kp_slider_sliderMoved(int position);

        void on_kp_slider_sliderPressed();

        void on_kp_slider_sliderReleased();

        void on_ki_slider_sliderPressed();

        void on_kd_slider_sliderPressed();

        void on_ki_slider_sliderReleased();

        void on_kd_slider_sliderReleased();

        void on_stopLog_pushButton_clicked();

        void on_update_slider_valueChanged(int value);

        void on_logSpeed_slider_valueChanged(int value);

        void on_logSpeed_slider_sliderPressed();

        void on_logSpeed_slider_sliderReleased();

    private:
        void getAvalilableSerialDevices();
        void serialRead();
        void serialWrite(QString message);
        int getChecksum(QString text);

        Ui::MainWindow *ui;

        QSerialPort *usbDevice;
        std::vector<QSerialPortInfo> serialComPortList; //A list of the available ports for the dropdownmenue in the GUI

        QString deviceDescription;
        QString serialBuffer;

        bool serialDeviceIsConnected;
        FILE *p_file;
        QTimer *timer;
        std::vector<std::string>    fileBuffer;
        unsigned int fileIndex;
        unsigned int updateSpeed;
        int activeSlider;
};

#endif // MAINWINDOW_H
