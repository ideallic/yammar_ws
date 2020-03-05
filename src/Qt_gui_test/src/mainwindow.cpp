#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    qnode(argc,argv)
{
    ui->setupUi(this);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
//    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButtonConnect_clicked()
{
    qnode.init();
}

void MainWindow::on_pushButtonSend_clicked()
{
    qnode.ros_test("I changed this world!");
}

void MainWindow::updateLogcamera()
{
  displayMat(qnode.image);
}

void MainWindow::displayMat(const QImage &image)
{
      qimage_mutex_.lock();
      qimage_ = image.copy();
      ui->label_camera->setPixmap(QPixmap::fromImage(qimage_.scaled(ui->label_camera->size(),Qt::KeepAspectRatio)));
      qimage_mutex_.unlock();
}
