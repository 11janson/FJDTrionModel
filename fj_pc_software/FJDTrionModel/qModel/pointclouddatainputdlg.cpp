#include "ui_pointclouddatainputdlg.h"
#include "pointclouddatainputdlg.h"
#include "framelessfiledialog.h"
#include <QStandardPaths>
#include <QPushButton>
#include <QLineEdit>
#include "cloudcompareutils\icore.h"
#include "framelessmessagebox.h"
PointCloudDataInputDlg::PointCloudDataInputDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::PointCloudDataInputDlg)
{
	m_ui->setupUi(this->getContentHolder());
    setWindowTitle(tr("Camera calibration"));
    this->bottomWidget()->setVisible(false);
    connect(m_ui->pushButton_fjdata, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectFJDataPath);
    connect(m_ui->pushButton_video, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectVideoDataPath);
    connect(m_ui->pushButton_fjdata_2, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectFJDataPath);
    connect(m_ui->pushButton_video_2, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectVideoDataPath);
    connect(m_ui->pushButton_fjdata_3, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectFJDataPath);
    connect(m_ui->pushButton_video_3, &QPushButton::clicked, this, &PointCloudDataInputDlg::selectVideoDataPath);
    connect(m_ui->pushButton_cancel, &QPushButton::clicked, this, &PointCloudDataInputDlg::reject);
    connect(m_ui->pushButton_ok, &QPushButton::clicked, this, &PointCloudDataInputDlg::nextStepApply);
}

static void getCloudPathFromName(QString objname, QString & pathname)
{
    int lastPosleft = objname.lastIndexOf("(");
    int lastPosright = objname.lastIndexOf(")");
    if (lastPosleft == -1 || lastPosright == -1 || lastPosleft > lastPosright)
    {
        return ;
    }
    QString subpathname = objname.mid(lastPosleft+1, lastPosright - lastPosleft-1);
    QString folderName = "/path/to/folder";
    QDir folder(subpathname);
    if (folder.exists()) 
    {
        pathname = subpathname;
    }
}

static void setLineEditStyle(QLineEdit * edit,bool isvalid)
{
    if (edit)
    {
        if (isvalid)
        {
            edit->setStyleSheet("border:1px solid #989898;background-color:#363636;color: #ffffff;border-radius: 2px;padding-left:12px;");
        }
        else
        {
            edit->setStyleSheet("border:1px solid #ff0000;background-color:#363636;color: #ffffff;border-radius: 2px;padding-left:12px;");
        }
    }
}

PointCloudDataInputDlg::~PointCloudDataInputDlg()
{
	delete m_ui;
}

void PointCloudDataInputDlg::nextStepApply()
{
    bool isok = true;
    if (m_cloudNum > 0)
    {
        if (m_ui->lineEdit_fjdata->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_fjdata, false);
            isok = false;
        }
        if (m_ui->lineEdit_videopath->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_videopath, false);
            isok = false;
        }

    }
    if (m_cloudNum > 1)
    {
        if (m_ui->lineEdit_fjdata_2->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_fjdata_2, false);
            isok = false;
        }
        if (m_ui->lineEdit_videopath_2->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_videopath_2, false);
            isok = false;
        }
    }
    if (m_cloudNum > 2)
    {
        if (m_ui->lineEdit_fjdata_3->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_fjdata_3, false);
            isok = false;
        }
        if (m_ui->lineEdit_videopath_3->text().isEmpty())
        {
            setLineEditStyle(m_ui->lineEdit_videopath_3, false);
            isok = false;
        }
    }
    if (isok)
    {
        accept();
    }

}

void PointCloudDataInputDlg::initOldData(std::vector<PointCloudData> data)
{
    m_cloudParentName.clear();
    m_cloudNum = data.size();
    if (data.size() > 0)
    {
        m_ui->lineEdit_cloudname->setText(data[0].cloudname);
        m_ui->lineEdit_fjdata->setText(data[0].fjdatapath);
        m_ui->lineEdit_videopath->setText(data[0].videopath);
        m_cloudParentName.push_back(data[0].parentobjname);
    }

    if (data.size() > 1)
    {
        m_ui->lineEdit_cloudname_2->setText(data[1].cloudname);
        m_ui->lineEdit_fjdata_2->setText(data[1].fjdatapath);
        m_ui->lineEdit_videopath_2->setText(data[1].videopath);
        m_cloudParentName.push_back(data[1].parentobjname);
    }

    if (data.size() > 2)
    {
        m_ui->lineEdit_cloudname_3->setText(data[2].cloudname);
        m_ui->lineEdit_fjdata_3->setText(data[2].fjdatapath);
        m_ui->lineEdit_videopath_3->setText(data[2].videopath);
        m_cloudParentName.push_back(data[2].parentobjname);
    }
    if (data.size() == 1)
    {
        m_ui->frame_2->setVisible(false);
        m_ui->frame_3->setVisible(false);
    }
    else if (data.size() == 2)
    {
        m_ui->frame_3->setVisible(false);
    }
}

void PointCloudDataInputDlg::initNewData(std::vector<PointCloudData> data)
{
    m_cloudParentName.clear();
    m_cloudNum = data.size();
    if (data.size() > 0)
    {
        m_ui->lineEdit_cloudname->setText(data[0].cloudname);
        m_cloudParentName.push_back(data[0].parentobjname);
    }

    if (data.size() > 1)
    {
        m_ui->lineEdit_cloudname_2->setText(data[1].cloudname);
        m_cloudParentName.push_back(data[1].parentobjname);
    }

    if (data.size() > 2)
    {
        m_ui->lineEdit_cloudname_3->setText(data[2].cloudname);
        m_cloudParentName.push_back(data[2].parentobjname);
    }
    if (data.size() == 1)
    {
        m_ui->frame_2->setVisible(false);
        m_ui->frame_3->setVisible(false);
    }
    else if (data.size() == 2)
    {
        m_ui->frame_3->setVisible(false);
    }
}

std::vector<PointCloudData> PointCloudDataInputDlg::getData()
{
    std::vector<PointCloudData> data;
    if (m_cloudNum > 0)
    {
        PointCloudData newdata;
        newdata.cloudname = m_ui->lineEdit_cloudname->text();
        newdata.fjdatapath = m_ui->lineEdit_fjdata->text();
        newdata.videopath = m_ui->lineEdit_videopath->text();
        newdata.parentobjname = m_cloudParentName[0];
        data.push_back(newdata);
    }
    if (m_cloudNum > 1)
    {
        PointCloudData newdata;
        newdata.cloudname = m_ui->lineEdit_cloudname_2->text();
        newdata.fjdatapath = m_ui->lineEdit_fjdata_2->text();
        newdata.videopath = m_ui->lineEdit_videopath_2->text();
        newdata.parentobjname = m_cloudParentName[1];
        data.push_back(newdata);
    }
    if (m_cloudNum > 2)
    {
        PointCloudData newdata;
        newdata.cloudname = m_ui->lineEdit_cloudname_3->text();
        newdata.fjdatapath = m_ui->lineEdit_fjdata_3->text();
        newdata.videopath = m_ui->lineEdit_videopath_3->text();
        newdata.parentobjname = m_cloudParentName[2];
        data.push_back(newdata);
    }
    return data;
}

void PointCloudDataInputDlg::selectFJDataPath()
{
    QString keystr = "";
    QPushButton* button = dynamic_cast<QPushButton*>(QObject::sender());
    if (button)
    {
        if (button->objectName() == "pushButton_fjdata")
        {
            keystr = m_cloudParentName[0];
        }
        else if (button->objectName() == "pushButton_fjdata_2")
        {
            keystr = m_cloudParentName[1];
        }
        else if (button->objectName() == "pushButton_fjdata_3")
        {
            keystr = m_cloudParentName[2];
        }
    }
    QString currentPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    getCloudPathFromName(keystr, currentPath);
    //QString currentPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "File(*.fjdata)");
    if (selectedFile.isEmpty())
    {
        return;
    }
    if (button)
    {
        if (button->objectName() == "pushButton_fjdata")
        {
            if (checkIsSameFjdata(selectedFile, m_ui->lineEdit_fjdata->objectName()))
            {
                return;
            }
            m_ui->lineEdit_fjdata->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_fjdata, true);
        }
        else if (button->objectName() == "pushButton_fjdata_2")
        {
            if (checkIsSameFjdata(selectedFile, m_ui->lineEdit_fjdata_2->objectName()))
            {
                return;
            }
            m_ui->lineEdit_fjdata_2->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_fjdata_2, true);
        }
        else if (button->objectName() == "pushButton_fjdata_3")
        {
            if (checkIsSameFjdata(selectedFile, m_ui->lineEdit_fjdata_3->objectName()))
            {
                return;
            }
            m_ui->lineEdit_fjdata_3->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_fjdata_3, true);
        }
    }
}

bool PointCloudDataInputDlg::checkIsSameFjdata(QString path, QString lineeditobjname)
{
    bool isfind = false;
    if (m_ui->lineEdit_fjdata->objectName() != lineeditobjname && m_ui->lineEdit_fjdata->text() == path)
    {
        isfind = true;
    }
    if (m_ui->lineEdit_fjdata_2->objectName() != lineeditobjname && m_ui->lineEdit_fjdata_2->text() == path)
    {
        isfind = true;
    }
    if (m_ui->lineEdit_fjdata_3->objectName() != lineeditobjname && m_ui->lineEdit_fjdata_3->text() == path)
    {
        isfind = true;
    }

    if (isfind)
    {
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Critical, tr("Error"), tr("Wrong config file selection. Config files with different gestures cannot be repeated."), QMessageBox::Ok, this);
        message_box.exec();
    }
    return isfind;
}

bool PointCloudDataInputDlg::checkIsSameVideodata(QString path, QString lineeditobjname)
{
    bool isfind = false;
    if (m_ui->lineEdit_videopath->objectName() != lineeditobjname && m_ui->lineEdit_videopath->text() == path)
    {
        isfind = true;
    }
    if (m_ui->lineEdit_videopath_2->objectName() != lineeditobjname && m_ui->lineEdit_videopath_2->text() == path)
    {
        isfind = true;
    }
    if (m_ui->lineEdit_videopath_3->objectName() != lineeditobjname && m_ui->lineEdit_videopath_3->text() == path)
    {
        isfind = true;
    }
    if (isfind)
    {
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Critical, tr("Error"), tr("Wrong video file selection. Video files with different gestures cannot be repeated."), QMessageBox::Ok, this);
        message_box.exec();
    }
    return isfind;
}

void PointCloudDataInputDlg::selectVideoDataPath()
{
    QString keystr = "Videopath";
    QPushButton* button = dynamic_cast<QPushButton*>(QObject::sender());
    if (button)
    {
        if (button->objectName() == "pushButton_video")
        {
            keystr = m_cloudParentName[0];
        }
        else if (button->objectName() == "pushButton_video_2")
        {
            keystr = m_cloudParentName[1];
        }
        else if (button->objectName() == "pushButton_video_3")
        {
            keystr = m_cloudParentName[2];
        }
    }
    QString currentPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    getCloudPathFromName(keystr, currentPath);
    QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "File(*.mp4)");
    if (selectedFile.isEmpty())
    {
        return;
    }
    if (button)
    {
        if (button->objectName() == "pushButton_video")
        {
            if (checkIsSameVideodata(selectedFile, m_ui->lineEdit_videopath->objectName()))
            {
                return;
            }
            m_ui->lineEdit_videopath->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_videopath, true);
        }
        else if (button->objectName() == "pushButton_video_2")
        {
            if (checkIsSameVideodata(selectedFile, m_ui->lineEdit_videopath_2->objectName()))
            {
                return;
            }
            m_ui->lineEdit_videopath_2->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_videopath_2, true);
        }
        else if (button->objectName() == "pushButton_video_3")
        {
            if (checkIsSameVideodata(selectedFile, m_ui->lineEdit_videopath_3->objectName()))
            {
                return;
            }
            m_ui->lineEdit_videopath_3->setText(selectedFile);
            setLineEditStyle(m_ui->lineEdit_videopath_3, true);
        }
    }
}

void PointCloudDataInputDlg::setIsUsePanoramicCamera(bool isues)
{
    m_ui->comboBox->setCurrentIndex(isues ? 0 : 1);
}


bool PointCloudDataInputDlg::getIsUsePanoramicCamera()
{
    return m_ui->comboBox->currentIndex() == 0;
}