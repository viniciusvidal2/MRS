#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>

#include <stdio.h>

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
  filename = QFileDialog::getOpenFileName(this, "Abrir arquivo", QDir::homePath(), "KML Files(*.kml)");
  ui->textEdit_nomearquivo->setText(filename);
}

void MainWindow::on_pushButton_converter_clicked()
{
  // Captar a altitude
  altitude = ui->textEdit_altura->toPlainText();
  if(altitude.isEmpty())
    altitude.fromStdString("750");

  // Comando chamando arquivo python com o nome do arquivo
  if(!filename.isEmpty()){
    std::string command = "gnome-terminal -x sh -c 'cd $HOME && python kml_waypoints.py "+filename.toStdString()+" "+altitude.toStdString()+"'";
    system(command.c_str());
    ui->label_fim->setText(QString::fromStdString("Conversao realizada, salva com o mesmo nome no diretorio"));
  } else {
    ui->label_fim->setText(QString::fromStdString("Escolha um arquivo primeiramente para conversao."));
  }
}

void MainWindow::on_pushButton_enviar_clicked()
{
    // Comando chamando arquivo python com o nome do arquivo
    ui->label_fim->setText(QString::fromStdString("entramos"));
    if(!filename.isEmpty()){
      std::string command = "gnome-terminal -x sh -c 'cd $HOME && python send_wp.py "+filename.toStdString()+"'";
      system(command.c_str());
      ui->label_fim->setText(QString::fromStdString("Aguarde o processo finalizar, entao os pontos ja estarao no sistema."));
    } else {
      ui->label_fim->setText(QString::fromStdString("Escolha um arquivo primeiramente para conversao."));
    }
}
