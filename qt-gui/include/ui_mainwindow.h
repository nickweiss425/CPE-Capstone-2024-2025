/********************************************************************************
** Form generated from reading UI file 'flight_controlANhoNd.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FLIGHT_CONTROLANHOND_H
#define FLIGHT_CONTROLANHOND_H

#include <QtCore/QVariant>
#include <QtQuickWidgets/QQuickWidget>
#include <QQmlContext>
#include <QQmlError>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "videowidget.hpp"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tabMap;
    QGridLayout *gridLayoutMap;
    QVBoxLayout *coordinateLayout;
    QLineEdit *latitudeBox;
    QLineEdit *longitudeBox;
    QHBoxLayout *altitudeLayout;
    QLineEdit *altitudeBox;
    QLineEdit *durationBox;
    QPushButton *hoverControlButton1;
    QPushButton *hoverControlButton2;
    QPushButton *hoverControlButton3;
    QPushButton *hoverControlButton4;
    QPushButton *confirmHoverButton;
    QHBoxLayout *buttonLayout;
    QPushButton *startFlightButton;
    QPushButton *toggleRecordingButton;
    QPushButton *stopFlightButton;
    QTextBrowser *sensorData;
    QQuickWidget *mapView;
    QWidget *tabVideo;
    QVBoxLayout *verticalLayoutVideo;
    VideoStreamWidget *videoWidget;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(932, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabMap = new QWidget();
        tabMap->setObjectName(QString::fromUtf8("tabMap"));
        gridLayoutMap = new QGridLayout(tabMap);
        gridLayoutMap->setObjectName(QString::fromUtf8("gridLayoutMap"));
        coordinateLayout = new QVBoxLayout();
        coordinateLayout->setObjectName(QString::fromUtf8("coordinateLayout"));
        latitudeBox = new QLineEdit(tabMap);
        latitudeBox->setObjectName(QString::fromUtf8("latitudeBox"));
        latitudeBox->setReadOnly(true);

        coordinateLayout->addWidget(latitudeBox);

        longitudeBox = new QLineEdit(tabMap);
        longitudeBox->setObjectName(QString::fromUtf8("longitudeBox"));
        longitudeBox->setReadOnly(true);
        longitudeBox->setInputMask(QString::fromUtf8("99.999"));

        coordinateLayout->addWidget(longitudeBox);


        gridLayoutMap->addLayout(coordinateLayout, 0, 1, 1, 1);

        altitudeLayout = new QHBoxLayout();
        altitudeLayout->setObjectName(QString::fromUtf8("altitudeLayout"));
        altitudeBox = new QLineEdit(tabMap);
        altitudeBox->setObjectName(QString::fromUtf8("altitudeBox"));

        altitudeLayout->addWidget(altitudeBox);

        durationBox = new QLineEdit(tabMap);
        durationBox->setObjectName(QString::fromUtf8("durationBox"));

        altitudeLayout->addWidget(durationBox);

        hoverControlButton1 = new QPushButton(tabMap);
        hoverControlButton1->setObjectName(QString::fromUtf8("hoverControlButton1"));

        altitudeLayout->addWidget(hoverControlButton1);

        hoverControlButton2 = new QPushButton(tabMap);
        hoverControlButton2->setObjectName(QString::fromUtf8("hoverControlButton2"));

        altitudeLayout->addWidget(hoverControlButton2);

        hoverControlButton3 = new QPushButton(tabMap);
        hoverControlButton3->setObjectName(QString::fromUtf8("hoverControlButton3"));

        altitudeLayout->addWidget(hoverControlButton3);

        hoverControlButton4 = new QPushButton(tabMap);
        hoverControlButton4->setObjectName(QString::fromUtf8("hoverControlButton4"));

        altitudeLayout->addWidget(hoverControlButton4);

        confirmHoverButton = new QPushButton(tabMap);
        confirmHoverButton->setObjectName(QString::fromUtf8("confirmHoverButton"));

        altitudeLayout->addWidget(confirmHoverButton);


        gridLayoutMap->addLayout(altitudeLayout, 2, 0, 1, 1);

        buttonLayout = new QHBoxLayout();
        buttonLayout->setObjectName(QString::fromUtf8("buttonLayout"));
        startFlightButton = new QPushButton(tabMap);
        startFlightButton->setObjectName(QString::fromUtf8("startFlightButton"));

        buttonLayout->addWidget(startFlightButton);

        toggleRecordingButton = new QPushButton(tabMap);
        toggleRecordingButton->setObjectName(QString::fromUtf8("toggleRecordingButton"));

        buttonLayout->addWidget(toggleRecordingButton);

        stopFlightButton = new QPushButton(tabMap);
        stopFlightButton->setObjectName(QString::fromUtf8("stopFlightButton"));

        buttonLayout->addWidget(stopFlightButton);


        gridLayoutMap->addLayout(buttonLayout, 2, 1, 1, 1);

        sensorData = new QTextBrowser(tabMap);
        sensorData->setObjectName(QString::fromUtf8("sensorData"));

        gridLayoutMap->addWidget(sensorData, 1, 1, 1, 1);

        mapView = new QQuickWidget(tabMap);
        mapView->setObjectName(QString::fromUtf8("mapView"));
        mapView->setResizeMode(QQuickWidget::SizeRootObjectToView);
        mapView->setSource(QUrl("qrc:/map.qml"));

        gridLayoutMap->addWidget(mapView, 0, 0, 2, 1);

        tabWidget->addTab(tabMap, QString());
        tabVideo = new QWidget();
        tabVideo->setObjectName(QString::fromUtf8("tabVideo"));
        verticalLayoutVideo = new QVBoxLayout(tabVideo);
        verticalLayoutVideo->setObjectName(QString::fromUtf8("verticalLayoutVideo"));
        videoWidget = new VideoStreamWidget(tabVideo);
        videoWidget->setObjectName(QString::fromUtf8("videoWidget"));

        verticalLayoutVideo->addWidget(videoWidget);

        tabWidget->addTab(tabVideo, QString());

        verticalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "NGCP Flight Control", nullptr));
        latitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Latitude", nullptr));
        longitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Longitude", nullptr));
        altitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Altitude", nullptr));
        durationBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Duration", nullptr));
        hoverControlButton1->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        hoverControlButton2->setText(QCoreApplication::translate("MainWindow", "C", nullptr));
        hoverControlButton3->setText(QCoreApplication::translate("MainWindow", "F8", nullptr));
        hoverControlButton4->setText(QCoreApplication::translate("MainWindow", "S", nullptr));
        confirmHoverButton->setText(QCoreApplication::translate("MainWindow", "\342\234\224", nullptr));
        startFlightButton->setText(QCoreApplication::translate("MainWindow", "Start Flight", nullptr));
        toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
        stopFlightButton->setText(QCoreApplication::translate("MainWindow", "Stop Flight", nullptr));
        toggleRecordingButton->setMaximumHeight(startFlightButton->sizeHint().height());
        sensorData->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tabMap), QCoreApplication::translate("MainWindow", "Map", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tabVideo), QCoreApplication::translate("MainWindow", "Video", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FLIGHT_CONTROLANHOND_H
