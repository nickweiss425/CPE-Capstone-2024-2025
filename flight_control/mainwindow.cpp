#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->mapView->setSource(QUrl("qrc:/map.qml"));

    ui->coordinateInputView->setSource(QUrl("qrc:/coordinatedisplay.qml"));
    ui->coordinateInputView->setResizeMode(QQuickWidget::SizeRootObjectToView);
}

MainWindow::~MainWindow()
{
    delete ui;
}
