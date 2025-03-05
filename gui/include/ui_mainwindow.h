// warning: nightmare below

#ifndef FLIGHT_CONTROLANHOND_H
#define FLIGHT_CONTROLANHOND_H

#include <QtCore/QVariant>
#include <QtQuickWidgets/QQuickWidget>
#include <QQuickItem>
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
#include <QButtonGroup>
#include "waypointmanager.hpp"

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
    QLineEdit *radiusBox;
    QLineEdit *altitudeBox;
    QLineEdit *durationBox;
    QButtonGroup *hoverGroup;
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
    WaypointManager *waypointManager;
    QLabel *radiusLabel;
    QLabel *altitudeLabel;
    QLabel *durationLabel;
    QLabel *connectionStatusLabel;

    void setupUi(QMainWindow *MainWindow)
    {
        // Create the main window with fixed initial size
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1150, 600);

        // Create the central widget
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));

        // Create the vertical layout for the central widget
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));

        // Create the tab widget
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);

        // Create the upper tab selection
        tabMap = new QWidget();
        tabMap->setObjectName(QString::fromUtf8("tabMap"));

        // Create the grid layout for the map tab
        gridLayoutMap = new QGridLayout(tabMap);
        gridLayoutMap->setObjectName(QString::fromUtf8("gridLayoutMap"));

        // Create the layout for the long, lat coordinates
        coordinateLayout = new QVBoxLayout();
        coordinateLayout->setObjectName(QString::fromUtf8("coordinateLayout"));

        connectionStatusLabel = new QLabel("CONNECTED", tabMap);
        connectionStatusLabel->setObjectName(QString::fromUtf8("connectionStatusLabel"));

        coordinateLayout->addWidget(connectionStatusLabel);

        // Create the text boxes for the long, lat coordinates
        latitudeBox = new QLineEdit(tabMap);
        latitudeBox->setObjectName(QString::fromUtf8("latitudeBox"));
        latitudeBox->setReadOnly(true);

        coordinateLayout->addWidget(latitudeBox);

        longitudeBox = new QLineEdit(tabMap);
        longitudeBox->setObjectName(QString::fromUtf8("longitudeBox"));
        longitudeBox->setReadOnly(true);

        coordinateLayout->addWidget(longitudeBox);

        // Add the coordinate layout to the grid layout
        gridLayoutMap->addLayout(coordinateLayout, 0, 1, 1, 1);

        // Create the layout for the altitude, radius, duration
        altitudeLayout = new QHBoxLayout();
        altitudeLayout->setObjectName(QString::fromUtf8("altitudeLayout"));

        radiusBox = new QLineEdit(tabMap);
        radiusBox->setObjectName(QString::fromUtf8("radiusBox"));

        radiusLabel = new QLabel("Radius:", tabMap);
        altitudeLayout->addWidget(radiusLabel);

        altitudeLayout->addWidget(radiusBox);

        altitudeBox = new QLineEdit(tabMap);
        altitudeBox->setObjectName(QString::fromUtf8("altitudeBox"));

        altitudeLabel = new QLabel("Altitude:", tabMap);
        altitudeLayout->addWidget(altitudeLabel);

        altitudeLayout->addWidget(altitudeBox);

        durationBox = new QLineEdit(tabMap);
        durationBox->setObjectName(QString::fromUtf8("durationBox"));

        durationLabel = new QLabel("Duration:", tabMap);
        altitudeLayout->addWidget(durationLabel);

        altitudeLayout->addWidget(durationBox);

        // Create the hover control buttons
        hoverGroup = new QButtonGroup(tabMap);
        hoverGroup->setObjectName(QString::fromUtf8("hoverGroup"));
        hoverGroup->setExclusive(true); // Only one hover control button can be selected at a time (mutually exclusive)

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

        // Add the hover control buttons to the button group
        hoverGroup->addButton(hoverControlButton1, 1);
        hoverGroup->addButton(hoverControlButton2, 2);
        hoverGroup->addButton(hoverControlButton3, 3);
        hoverGroup->addButton(hoverControlButton4, 4);

        altitudeLayout->addWidget(hoverControlButton4);

        // Create the confirm hover button
        confirmHoverButton = new QPushButton(tabMap);
        confirmHoverButton->setObjectName(QString::fromUtf8("confirmHoverButton"));

        altitudeLayout->addWidget(confirmHoverButton);

        // Create the waypoint manager
        waypointManager = new WaypointManager(tabMap);
        waypointManager->setObjectName(QString::fromUtf8("waypointManager"));

        gridLayoutMap->addLayout(altitudeLayout, 2, 0, 1, 1);

        buttonLayout = new QHBoxLayout();

        // Create the buttons for starting, stopping, and recording the flight
        toggleRecordingButton = new QPushButton(tabMap);
        toggleRecordingButton->setObjectName(QString::fromUtf8("toggleRecordingButton"));

        buttonLayout->addWidget(toggleRecordingButton);

        stopFlightButton = new QPushButton(tabMap);
        stopFlightButton->setObjectName(QString::fromUtf8("stopFlightButton"));

        buttonLayout->addWidget(stopFlightButton);

        buttonLayout->setObjectName(QString::fromUtf8("buttonLayout"));
        startFlightButton = new QPushButton(tabMap);
        startFlightButton->setObjectName(QString::fromUtf8("startFlightButton"));

        buttonLayout->addWidget(startFlightButton);

        gridLayoutMap->addLayout(buttonLayout, 2, 1, 1, 1);

        // Create the sensor data text browser
        sensorData = new QTextBrowser(tabMap);
        sensorData->setObjectName(QString::fromUtf8("sensorData"));

        gridLayoutMap->addWidget(sensorData, 1, 1, 1, 1);

        // Create the map view and load qml file
        mapView = new QQuickWidget(tabMap);
        mapView->setObjectName(QString::fromUtf8("mapView"));
        mapView->setResizeMode(QQuickWidget::SizeRootObjectToView);
        mapView->rootContext()->setContextProperty("mapView", mapView);
        mapView->setSource(QUrl("qrc:/map.qml"));

        gridLayoutMap->addWidget(mapView, 0, 0, 2, 1);

        // Change fullscreen scaling to maintain reasonable widget sizes (75% map, 25% sensor data)
        gridLayoutMap->setColumnStretch(0, 3);
        gridLayoutMap->setColumnStretch(1, 1);

        // Add the tabs to the tab widget
        tabWidget->addTab(tabMap, QString());
        tabVideo = new QWidget();
        tabVideo->setObjectName(QString::fromUtf8("tabVideo"));
        verticalLayoutVideo = new QVBoxLayout(tabVideo);
        verticalLayoutVideo->setObjectName(QString::fromUtf8("verticalLayoutVideo"));
        videoWidget = new VideoStreamWidget(tabVideo);
        videoWidget->setObjectName(QString::fromUtf8("videoWidget"));

        // Add the video widget to the video tab
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
        // set the window title and button text
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "NGCP Flight Control", nullptr));
        latitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Latitude", nullptr));
        longitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "Longitude", nullptr));
        radiusBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "meters", nullptr));
        altitudeBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "meters", nullptr));
        durationBox->setPlaceholderText(QCoreApplication::translate("MainWindow", "seconds", nullptr));
        hoverControlButton1->setText(QCoreApplication::translate("MainWindow", "Loiter", nullptr));
        hoverControlButton2->setText(QCoreApplication::translate("MainWindow", "Circle", nullptr));
        hoverControlButton3->setText(QCoreApplication::translate("MainWindow", "Figure-8", nullptr));
        hoverControlButton4->setText(QCoreApplication::translate("MainWindow", "Square", nullptr));
        confirmHoverButton->setText(QCoreApplication::translate("MainWindow", "\342\234\224", nullptr));
        confirmHoverButton->setStyleSheet(QString::fromUtf8("background-color: rgb(36, 182, 36);"));
        startFlightButton->setText(QCoreApplication::translate("MainWindow", "Start Flight", nullptr));
        toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
        toggleRecordingButton->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 0, 0);"));
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
