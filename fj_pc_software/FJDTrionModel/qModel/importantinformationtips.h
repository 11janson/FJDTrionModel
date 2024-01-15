#pragma once
#include <QObject>
#include<QApplication>
#include "mainwindow.h"
class InformationPrompt
{
public:
    InformationPrompt(QString positionstr,QObject *parent = nullptr);
    ~InformationPrompt()
    {
        MainWindow::TheInstance()->setStatusTip(QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr));
    }
};
InformationPrompt::InformationPrompt(QString positionstr, QObject *parent)
{
    QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr);
    MainWindow::TheInstance()->statusBar()->showMessage(positionstr);
}

