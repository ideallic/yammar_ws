#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    qnode(argc,argv)
{
    ui->setupUi(this);
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

void MainWindow::on_pushButton_clicked()
{
    qnode.init();
    qnode.ros_test("I changed this world again!");
}
