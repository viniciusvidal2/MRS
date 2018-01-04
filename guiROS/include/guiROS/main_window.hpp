/**
 * @file /include/guiROS/main_window.hpp
 *
 * @brief Qt based gui for guiROS.
 *
 *
 **/
#ifndef guiROS_MAIN_WINDOW_H
#define guiROS_MAIN_WINDOW_H

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

#include <iostream>

using namespace cv;
using namespace std;

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace guiROS {

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
        void on_pushButton_open_webcam_clicked();
        void on_pushButton_close_webcam_clicked();
        void update_window();
        void receive_mat_image(cv::Mat img,qint64 timestamp);

private:
        Ui::MainWindowDesign ui;

        QNode qnode;
        GigeImageReader gige_ir;
        QTimer *timer;
        VideoCapture cap;
        Mat frame;
        QImage qt_image;

};

}  // namespace guiROS

#endif // guiROS_MAIN_WINDOW_H
