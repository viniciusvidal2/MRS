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
#include "../include/monitor_mrs/main_window.hpp"

#include <syscall.h>

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
    monitor_mrs::MainWindow w(argc,argv);

    system("gnome-terminal -x sh -c 'roscore'");

    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();



	return result;
}
