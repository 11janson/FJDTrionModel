//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef __FJSECTIONANALYSISDLG_H__
#define __FJSECTIONANALYSISDLG_H__

//Local
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"

//qCC_db
#include <ccPointCloud.h>
#include <QObject>
#include <ui_fjsectionanalysisdlg.h>
#include "framelessdialog.h"
#include <QTableWidgetItem>
#include <QCloseEvent>
#include <QVBoxLayout>
#include <QCheckBox>
#include "ccSectionAnalysisLine.h"
#include "ccPolyline.h"
#include "cloudcomparewidgets\metahubframelesswaitdialog.h"
class ccHObject;
class ccGenericPointCloud;
class ccGenericGLDisplay;
class ccGLWindow;
class cc2DLabel;
class ccPickingHub;

enum SectionViewMode
{
    VIEW3DMODE,
    TOPVIEWMODE,
    FRONTVIEWMODE
};

class FJSectionAnalysisDlg : public CS::Widgets::FramelessDialog, public ccPickingListener, Ui::FJSectionAnalysisDlg
{
    Q_OBJECT

public:

    //! Default constructor
    explicit FJSectionAnalysisDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent = nullptr);
    ~FJSectionAnalysisDlg();
    //inherited from ccOverlayDialog
    bool linkWith(ccGLWindow* win);
    bool start();
    void stop(bool state);

    //! Inits dialog
    bool init(ccGLWindow* win, const ccHObject::Container& alignedEntities);

    //! Inherited from ccPickingListener
    void onItemPicked(const PickedItem& pi) override;

	//计算点云投影点
	void calculateSectionAnalysisData(QString name,double thickness,double offset, bool isnewfile,QString direction,bool updateoffsetdir = true);

    //更新点大小
    void updatePointSize();

signals:
    void processFinshed(bool isclose);
public slots:
    void stopPickPoint();

    void updateCloudSelected(QTableWidgetItem *item, bool updateoffsetdir = true);

    //更新右上角标签显示
    void updateLabelText(QString str);

    //更新剖面厚度
    void updateThick();

    //开始测量
    void startMeasure();

    //结束选点
    void onFinshedPickPoint(QPointF start, QPointF end);

    //删除剖面
    void onDelButtonPushed();

    //更新鼠标位置
    void updateMousePos(int x, int y, Qt::MouseButtons buttons);

    void setView3d();

    void setViewUp();

    void setViewFace();

    QString getCurrentSectionName();

    void onThicknessOrOffsetChanged(double value);


    void setViewChangeButtonEnable(bool isenable);

    void onSectionThicknessUpdate(QString name, double thick);

    void addPointToline(int x, int y);

    //点云勾选槽函数
    void checkboxStateChanged(int);

    //调整窗口大小
    void slotUpdateWindowSize();

    void updatePointColor();
	
    void slotCurrentItemSelectChanged();

protected:
    void updateData();
    virtual void closeEvent(QCloseEvent *event);
    bool eventFilter(QObject *obj, QEvent *e) override;

    //计算选中点云偏移范围和高度范围
    void calculateWidthAndHeightRange();

protected: //members

    ccHObject* m_object;

    ccHObject::Container  m_selectedObjectList;

    //! Dedicated window
    ccGLWindow* m_win;

    //! Picking hub
    ccPickingHub* m_pickingHub;

    //! Main application interface
    ccMainAppInterface* m_app;

    std::vector<CCVector3d> m_CurrentPointData;

    ccSectionAnalysisLine * m_Line;

    std::map<QString, SectionAnalysisData> m_SectionAnalysisData;
    int m_currentSectionNum = -1;

    int m_currentSectionVerticalNum = -1;
    int m_currentSectionHorizontalNum = -1;

    double m_heightMin = 99999999;
    double m_heightMax = -99999999;
    double m_xMax = 0;

    double m_heightMinY = 99999999;
    double m_heightMaxY = -99999999;
    double m_zMax = 0;
    ProjectionDirection m_ProjectionMode = LOOKDOWN;
    int m_pointSum = 0;
    SectionViewMode m_viewMode = TOPVIEWMODE;
    WaitingDialog::MetahublFramelessWaitingdialog * m_Waitingdlg;

    QVBoxLayout *m_layout; //点云布局框

    std::vector<QCheckBox *> m_checkBoxs; //当前点云勾选框

    bool m_showPointColor = false;//是否显示点云颜色
};

#endif //POINT_PAIR_REGISTRATION_DIALOG_HEADER
