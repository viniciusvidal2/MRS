/**
 * @file /include/guiROS/main_window.hpp
 *
 * @brief Qt based gui for guiROS.
 *
 *
 **/
#ifndef monitor_mrs_MAIN_WINDOW_H
#define monitor_mrs_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include "GigeImageReader.hpp"
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QTimer>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/videoio.hpp"

#include <iostream>

#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
//#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>

using namespace cv;
using namespace std;

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace monitor_mrs {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  QMutex mutex;
	~MainWindow();

        void closeEvent(QCloseEvent *event);


public Q_SLOTS:


private Q_SLOTS:
        void update_window();
        void receive_mat_image(cv::Mat img,qint64 timestamp);
        int getProcIdByName(string procName);
        // Vinicius, aba 1
        void on_pushButton_motores_clicked();
        void on_pushButton_qground_clicked();
        void on_pushButton_resetaPX4_clicked();
        void on_pushButton_iniciaStereo_clicked();
        void on_pushButton_salvaBag_clicked();
        void on_pushButton_nuvemInstantanea_clicked();
        void on_pushButton_reiniciarTudo_clicked();
        void on_pushButton_limpaTexto_clicked();

private:
        Ui::MainWindowDesign ui;
        std::string nome;

        QNode qnode;
        GigeImageReader gige_ir;
        QTimer *timer;
        VideoCapture cap;
        Mat frame;
        QImage qt_image;

        bool controle_gravacao;
};

}  // namespace monitor_mrs

#endif // monitor_mrs_MAIN_WINDOW_H
