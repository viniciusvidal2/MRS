/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTextEdit *textEdit_nomearquivo;
    QPushButton *pushButton_escolherarquivo;
    QPushButton *pushButton_converter;
    QLabel *label_fim;
    QTextEdit *textEdit_altura;
    QLabel *label_fim_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(754, 198);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        textEdit_nomearquivo = new QTextEdit(centralWidget);
        textEdit_nomearquivo->setObjectName(QString::fromUtf8("textEdit_nomearquivo"));
        textEdit_nomearquivo->setGeometry(QRect(150, 0, 581, 31));
        pushButton_escolherarquivo = new QPushButton(centralWidget);
        pushButton_escolherarquivo->setObjectName(QString::fromUtf8("pushButton_escolherarquivo"));
        pushButton_escolherarquivo->setGeometry(QRect(10, 0, 131, 31));
        pushButton_converter = new QPushButton(centralWidget);
        pushButton_converter->setObjectName(QString::fromUtf8("pushButton_converter"));
        pushButton_converter->setGeometry(QRect(10, 90, 347, 25));
        label_fim = new QLabel(centralWidget);
        label_fim->setObjectName(QString::fromUtf8("label_fim"));
        label_fim->setGeometry(QRect(370, 90, 341, 25));
        textEdit_altura = new QTextEdit(centralWidget);
        textEdit_altura->setObjectName(QString::fromUtf8("textEdit_altura"));
        textEdit_altura->setGeometry(QRect(200, 40, 531, 31));
        label_fim_2 = new QLabel(centralWidget);
        label_fim_2->setObjectName(QString::fromUtf8("label_fim_2"));
        label_fim_2->setGeometry(QRect(10, 40, 171, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 754, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        pushButton_escolherarquivo->setText(QApplication::translate("MainWindow", "Arquivo KML", 0, QApplication::UnicodeUTF8));
        pushButton_converter->setText(QApplication::translate("MainWindow", "Converter para .waypoint", 0, QApplication::UnicodeUTF8));
        label_fim->setText(QString());
        label_fim_2->setText(QApplication::translate("MainWindow", "Altitude local estimada", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
