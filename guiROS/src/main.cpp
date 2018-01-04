/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QApplication>
#include "../include/guiROS/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /***********************
     * ROS
     * *******************/


    /*********************
    ** Qt
    **********************/

    QApplication app(argc, argv);
    guiROS::MainWindow w(argc,argv);

    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();

	return result;
}
