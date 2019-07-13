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
#include <QFileDialog>
#include <QLabel>

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
        void on_pushButton_cameratermica_clicked();
        void on_pushButton_reconstrucaoInstantaneaTermica_clicked();
        void on_pushButton_salvaBag_clicked();
        void on_pushButton_nuvemInstantanea_clicked();
        void on_pushButton_reiniciarTudo_clicked();
        void on_pushButton_limpaTexto_clicked();
        void on_horizontalSlider_offset_sliderMoved();
        void on_verticalSlider_offset_sliderMoved();
        void on_pushButton_enviaraio_clicked();
        void on_radioButton_pontosdeinteresse_clicked();        
        void on_radioButton_caminhocompleto_clicked();

        //Lucas, aba 2
        void on_pushButtonSelectBag_clicked();
        void on_pushButton_playBag_clicked();
        void on_pushButton_clear2_clicked();
        void on_pushButton_paraBag_clicked();
        void on_pushButton_salvaNuvem_clicked();
        void on_pushButton_recAcumulada_clicked();
        void on_pushButton_recAcumuladaTermica_clicked();

        void on_pushButton_setaimagem_clicked();

private:
        Ui::MainWindowDesign ui;
        std::string nome;

        QNode qnode;
        GigeImageReader gige_ir;
        QTimer *timer;
        VideoCapture cap;
        Mat frame;
        QImage qt_image;

        bool controle_stereo;
        bool controle_gravacao;
        int offset_pan, offset_tilt;
        int esquema_apontar_caminho; // 1 = apontar so em frente; 2 = apontar para os pontos de interesse automaticamente

        enum controle_imagem_subscrever {visual, termica};
        controle_imagem_subscrever fonte_imagem;

        // Lucas, aba 2
        QString filename;
        std::string localstd;
};

}  // namespace monitor_mrs

#endif // monitor_mrs_MAIN_WINDOW_H
