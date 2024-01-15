#include"metahubpdfdialog.h"
#include<QCoreApplication>
#include "qpdfwidget.h"
#include "libs/cloudcompareutils/icore.h"
#include <QDebug>
CS::Widgets::pdfDialog::pdfDialog(QWidget *parent)
    : CS::Widgets::FramelessDialog(parent)
{
    setPdfStyleSheet();
}
void CS::Widgets::pdfDialog::setPdfStyleSheet()
{
    QPdfWidget * pdfWidget = new QPdfWidget(this);
    if (!pdfWidget->loadFile(QCoreApplication::applicationDirPath() + "/share/doc/helpdoc" + CS::Core::ICore::getOverrideLanguage() + ".pdf"))
    {
        delete pdfWidget;
        qDebug() << "not found helppdf file";
        return;
    }
    qApp->setProperty("document", true);
    this->SetContentHolder(pdfWidget);
    this->setAttribute(Qt::WA_DeleteOnClose, true);
    this->resize(1280, 968);
    QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/minimize.png");
    this->getTitleBar()->setTitleButtonIcon(1, pIcon);
    QIcon pIcons(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png");//btn_max.png
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png"), QIcon::Active, QIcon::Off);
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png"), QIcon::Disabled, QIcon::Off);
    this->getTitleBar()->setTitleButtonIcon(2, pIcons);
    QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
    this->getTitleBar()->setTitleButtonIcon(4, closeicon);
    this->setMinButtonVisible(true);
    // 显示最大化按钮
    this->setMaxButtonVisible(true);
    this->setDialogResizable(true);
    this->setWindowTitle(QCoreApplication::translate("MainWindowUI", "Help Document", nullptr));
    this->getTitleBar()->setStyleSheet("font-size:16px;color: #F2F2F2;background-color:rgba(69, 69, 69, 1);");
    this->setTtitleBarButtonStyle(TitleBar::PredefinedButtonType::MinButton, "background-color:rgba(69, 69, 69, 1);");
    this->setTtitleBarButtonStyle(TitleBar::PredefinedButtonType::MaxButton, "background-color:rgba(69, 69, 69, 1);");
    this->setTtitleBarButtonStyle(TitleBar::PredefinedButtonType::CloseButton, "background-color:rgba(69, 69, 69, 1);");
    this->setTtitleBarButtonStyle(TitleBar::PredefinedButtonType::RestoreButton, "background-color:rgba(69, 69, 69, 1);");
    this->bottomWidget()->setVisible(false);
}

CS::Widgets::pdfDialog::~pdfDialog()
{
    qApp->setProperty("document", false);
}