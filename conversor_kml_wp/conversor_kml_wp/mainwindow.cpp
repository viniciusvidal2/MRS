#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_escolherarquivo_clicked()
{
  filename = QFileDialog::getOpenFileName(this, "Abrir arquivo", "", "KML Files(*.kml)");
  ui->textEdit_nomearquivo->setText(filename);
}

void MainWindow::on_pushButton_converter_clicked()
{
  system("gnome-terminal -x sh -c 'cd $HOME && roslaunch automatico_mrs lancar_gimbal.launch'");
}

