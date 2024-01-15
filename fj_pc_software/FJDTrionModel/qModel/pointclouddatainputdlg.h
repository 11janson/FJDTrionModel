#ifndef __POINTCLOUDDATAINPUTDLG_H__
#define __POINTCLOUDDATAINPUTDLG_H__

#include <QDialog>
#include "framelessdialog.h"

namespace Ui {
	class PointCloudDataInputDlg;
}

struct PointCloudData
{
    int cloudid = -1;
    QString cloudname = "";
    QString fjdatapath = "";
    QString videopath = "";
    QString parentobjname = "";
};

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class PointCloudDataInputDlg : public CS::Widgets::FramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit PointCloudDataInputDlg(QWidget* parent = nullptr);	

    ~PointCloudDataInputDlg();
	
    //旧数据初始化界面参数
    void initOldData(std::vector<PointCloudData> data);

    //新数据初始化界面参数
    void initNewData(std::vector<PointCloudData> data);

    //获取界面信息
    std::vector<PointCloudData> getData();

    //检查是否存在相同的fjdata
    bool checkIsSameFjdata(QString path,QString lineeditobjname);

    //检查是否存在相同的mp4
    bool checkIsSameVideodata(QString path, QString lineeditobjname);

    //设置是否使用全景相机
    void setIsUsePanoramicCamera(bool isues);

    //获取是否使用全景相机
    bool getIsUsePanoramicCamera();

public slots:

    //选择fjdata数据
    void selectFJDataPath();

    //选择MP4数据
    void selectVideoDataPath();

    //下一步按钮
    void nextStepApply();

private:
	Ui::PointCloudDataInputDlg* m_ui;

    int m_cloudNum = 0;

    std::vector<QString> m_cloudParentName;
};

#endif //CC_SOR_FILTER_DLG_HEADER
