/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.11.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QComboBox *serialPortSelect_comboBox;
    QPushButton *connect_button;
    QPushButton *disconnect_button;
    QPushButton *pushButton;
    QSlider *kp_slider;
    QSlider *ki_slider;
    QSlider *kd_slider;
    QLabel *p_label;
    QLabel *i_label;
    QLabel *d_label;
    QLabel *label;
    QPushButton *stopLog_pushButton;
    QSpinBox *fileIndex_spinbox;
    QSlider *update_slider;
    QSlider *logSpeed_slider;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(400, 300);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        serialPortSelect_comboBox = new QComboBox(centralWidget);
        serialPortSelect_comboBox->setObjectName(QStringLiteral("serialPortSelect_comboBox"));
        serialPortSelect_comboBox->setGeometry(QRect(10, 10, 261, 22));
        connect_button = new QPushButton(centralWidget);
        connect_button->setObjectName(QStringLiteral("connect_button"));
        connect_button->setGeometry(QRect(280, 10, 75, 23));
        disconnect_button = new QPushButton(centralWidget);
        disconnect_button->setObjectName(QStringLiteral("disconnect_button"));
        disconnect_button->setGeometry(QRect(280, 40, 75, 23));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(280, 80, 75, 23));
        kp_slider = new QSlider(centralWidget);
        kp_slider->setObjectName(QStringLiteral("kp_slider"));
        kp_slider->setGeometry(QRect(20, 60, 22, 160));
        kp_slider->setMaximum(1000);
        kp_slider->setOrientation(Qt::Vertical);
        ki_slider = new QSlider(centralWidget);
        ki_slider->setObjectName(QStringLiteral("ki_slider"));
        ki_slider->setGeometry(QRect(60, 60, 22, 160));
        ki_slider->setMaximum(1000);
        ki_slider->setOrientation(Qt::Vertical);
        kd_slider = new QSlider(centralWidget);
        kd_slider->setObjectName(QStringLiteral("kd_slider"));
        kd_slider->setGeometry(QRect(100, 60, 22, 160));
        kd_slider->setMaximum(1000);
        kd_slider->setOrientation(Qt::Vertical);
        p_label = new QLabel(centralWidget);
        p_label->setObjectName(QStringLiteral("p_label"));
        p_label->setGeometry(QRect(20, 220, 21, 20));
        i_label = new QLabel(centralWidget);
        i_label->setObjectName(QStringLiteral("i_label"));
        i_label->setGeometry(QRect(60, 220, 21, 20));
        d_label = new QLabel(centralWidget);
        d_label->setObjectName(QStringLiteral("d_label"));
        d_label->setGeometry(QRect(100, 220, 21, 20));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 40, 101, 20));
        QFont font;
        font.setPointSize(13);
        label->setFont(font);
        stopLog_pushButton = new QPushButton(centralWidget);
        stopLog_pushButton->setObjectName(QStringLiteral("stopLog_pushButton"));
        stopLog_pushButton->setGeometry(QRect(280, 110, 75, 23));
        fileIndex_spinbox = new QSpinBox(centralWidget);
        fileIndex_spinbox->setObjectName(QStringLiteral("fileIndex_spinbox"));
        fileIndex_spinbox->setGeometry(QRect(360, 80, 42, 22));
        update_slider = new QSlider(centralWidget);
        update_slider->setObjectName(QStringLiteral("update_slider"));
        update_slider->setGeometry(QRect(240, 50, 22, 160));
        update_slider->setMaximum(1000);
        update_slider->setValue(500);
        update_slider->setOrientation(Qt::Vertical);
        logSpeed_slider = new QSlider(centralWidget);
        logSpeed_slider->setObjectName(QStringLiteral("logSpeed_slider"));
        logSpeed_slider->setGeometry(QRect(200, 50, 22, 160));
        logSpeed_slider->setMinimum(1);
        logSpeed_slider->setMaximum(1000);
        logSpeed_slider->setValue(500);
        logSpeed_slider->setOrientation(Qt::Vertical);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 21));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        connect_button->setText(QApplication::translate("MainWindow", "connect", nullptr));
        disconnect_button->setText(QApplication::translate("MainWindow", "disconnect", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "startLog", nullptr));
        p_label->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        i_label->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        d_label->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        label->setText(QApplication::translate("MainWindow", "P       I       D", nullptr));
        stopLog_pushButton->setText(QApplication::translate("MainWindow", "stopLog", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
