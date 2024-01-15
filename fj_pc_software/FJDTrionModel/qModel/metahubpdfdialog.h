#pragma once

#include"framelessdialog.h"
#include"titleBar.h"
#include<QIcon>
#include"FJStyleManager.h"
namespace CS::Widgets
{
    class pdfDialog : public CS::Widgets::FramelessDialog
    {
    public:
        explicit pdfDialog(QWidget *parent = nullptr);
        ~pdfDialog();
    public:
        void setPdfStyleSheet();
    };
}

