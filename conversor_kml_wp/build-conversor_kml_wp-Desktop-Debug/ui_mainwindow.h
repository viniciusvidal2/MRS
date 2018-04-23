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
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTextEdit *textEdit_nomearquivo;
    QPushButton *pushButton_escolherarquivo;
    QLabel *label_fim;
    QTextEdit *textEdit_altura;
    QLabel *label_altura;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_converter;
    QPushButton *pushButton_enviar;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1136, 344);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        textEdit_nomearquivo = new QTextEdit(centralWidget);
        textEdit_nomearquivo->setObjectName(QString::fromUtf8("textEdit_nomearquivo"));
        textEdit_nomearquivo->setGeometry(QRect(150, 0, 971, 31));
        pushButton_escolherarquivo = new QPushButton(centralWidget);
        pushButton_escolherarquivo->setObjectName(QString::fromUtf8("pushButton_escolherarquivo"));
        pushButton_escolherarquivo->setGeometry(QRect(10, 0, 131, 31));
        label_fim = new QLabel(centralWidget);
        label_fim->setObjectName(QString::fromUtf8("label_fim"));
        label_fim->setGeometry(QRect(370, 90, 751, 151));
        textEdit_altura = new QTextEdit(centralWidget);
        textEdit_altura->setObjectName(QString::fromUtf8("textEdit_altura"));
        textEdit_altura->setGeometry(QRect(200, 40, 921, 31));
        label_altura = new QLabel(centralWidget);
        label_altura->setObjectName(QString::fromUtf8("label_altura"));
        label_altura->setGeometry(QRect(10, 40, 171, 25));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 90, 341, 58));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_converter = new QPushButton(layoutWidget);
        pushButton_converter->setObjectName(QString::fromUtf8("pushButton_converter"));

        verticalLayout->addWidget(pushButton_converter);

        pushButton_enviar = new QPushButton(layoutWidget);
        pushButton_enviar->setObjectName(QString::fromUtf8("pushButton_enviar"));

        verticalLayout->addWidget(pushButton_enviar);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1136, 22));
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
        label_fim->setText(QString());
        label_altura->setText(QApplication::translate("MainWindow", "Altitude local estimada", 0, QApplication::UnicodeUTF8));
        pushButton_converter->setText(QApplication::translate("MainWindow", "Converter para .waypoint", 0, QApplication::UnicodeUTF8));
        pushButton_enviar->setText(QApplication::translate("MainWindow", "Enviar para o controle e navegacao", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
