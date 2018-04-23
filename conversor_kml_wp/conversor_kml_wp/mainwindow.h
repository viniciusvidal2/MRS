#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>

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
//  void on_pushButton_escolherarquivo_clicked(bool checked);
  void on_pushButton_converter_clicked();
  void on_pushButton_escolherarquivo_clicked();


  void on_pushButton_enviar_clicked();

private:
    Ui::MainWindow *ui;

    QString filename;
    QString altitude;
};

#endif // MAINWINDOW_H
