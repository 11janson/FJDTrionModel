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

#ifndef POINT_PAIR_REGISTRATION_DIALOG_HEADER
#define POINT_PAIR_REGISTRATION_DIALOG_HEADER

//Local
#include "ccMainAppInterface.h"
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"
#include "metahubtoolbutton.h"
#include "ccRegistrationDlg.h"
#include "ccRegistrationTools.h"
//CCCoreLib
#include <PointProjectionTools.h>

//qCC_db
#include <ccPointCloud.h>

//Qt generated dialog
#include <ui_pointPairRegistrationDlg.h>
#include "libs\cloudcomparewidgets\metahublframelessdialog.h"
//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class ccGenericPointCloud;
class ccGenericGLDisplay;
class ccGLWindow;
class cc2DLabel;
class ccPickingHub;
class CheckBoxHeaderView;
class constituencyRegistration;
//Dialog for the point-pair registration algorithm (Horn)
class ccPointPairRegistrationDlg : public ccOverlayDialog, public ccPickingListener, Ui::pointPairRegistrationDlg
{
    Q_OBJECT

public:

    struct HistoryData
    {

        int AlignedPointsUuid = -1;
        int RefPointsUuid = -1;
        std::vector<std::vector<QString>>  AlignedPointsdata;
        std::vector<std::vector<QString>>  RefPointsdata;
        std::vector<bool> AlignedPointsCheckstate;
        std::vector<bool> RefPointsCheckstate;
    };

    enum CurrentPickMode {
        NOPICK = 0,
        PICKPOINT,
        PICKPAPER,
        PICKBALL
    };

    /**
    *@brief 设置是否显示拾取点
    */
    void setPickPointMode(bool isShow);

    /**
    *@brief 设置为坐标转换样式
    */
    void setCoordinateConversionStyle(bool isshow);


    //! Default constructor
    explicit ccPointPairRegistrationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent = nullptr);

    void onShortcutDoubleBondTriggered(int key, Qt::KeyboardModifiers modifiers);
    //inherited from ccOverlayDialog
    bool linkWith(ccGLWindow* win) override;
    bool start() override;
    void stop(bool state) override;

    //! Inits dialog
    bool init(ccGLWindow* win,
        const ccHObject::Container& alignedEntities,
        const ccHObject::Container* referenceEntities = nullptr);

    //! Clears dialog
    void clear();

    //! Pauses the dialog
    void pause(bool state);

    //! Adds a point to the 'align' set
    bool addAlignedPoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true, bool isaddname = false, QString idname = nullptr);
    //! Adds a point to the 'reference' set
    bool addReferencePoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true, bool isaddname = false, QString idname = nullptr);

    //! Inherited from ccPickingListener
    void onItemPicked(const PickedItem& pi) override;


    void changeLabelName(ccPointCloud entity, int index, QString newName);

    //[!]记录是否是在组合模式下关闭裁剪盒
    bool recordCombinationWhether = false;
    //[!]记录是否是组合模式
    bool m_bCurrentModelCombination = false;
    //[!]重设组合模式下，点云状态
    void setPointPairRegistrationCurrentButtonstate();

    //[!]切换配准的功能
    void switchingRegistrationMethods();

    //[!]设置界面的2dlabel显示
    void clearLabel(bool);

signals:

    void showMessage(QString mes);

    void signalPreview();//预览

    void signalReset();//重置

    void signalOctreeDeletion(bool isCoarse);

public slots:

    //we temporarily detach entity, as it may undergo
    //"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
    void slotOctreeDeletion(bool isCoarse);

    //! Updates the registration info and buttons states
    void updateAlignInfo();

    void apply();
    void align();
    void reset();
    void cancel();


    void addSource();
    void importSource();
    void upSource();
    void downSource();
    void deleteSource();
    void addTarget();
    void importTarget();
    void upTarget();
    void downTarget();
    void deleteTarget();
    void exportSource();
    void exportTarget();

    //设置ICP配准
    void getIcpSetting();

    //从txt读取转化矩阵与缩放因子
    void getTransformationMatrixFromTxt();

    //Aligned表头勾选框状态变化
    void slotAlignedHeadviewerStateChanged(bool checked);

    //Ref表头勾选框状态变化
    void slotRefHeadviewerStateChanged(bool checked);

    //Aligne表选择变化
    void slotalignedItemSelectionChanged();

    //Ref表选择变化
    void slotRefItemSelectionChanged();

    //Aligne表内容变化
    void slotaligneditemChanged(QTableWidgetItem *item);

    //Ref表内容变化
    void slotRefitemChanged(QTableWidgetItem *item);

    //拾取按钮点击
    void slotPickToolButtonClicked();

    //靶标球按钮点击
    void slotBallToolButtonClicked();

    //靶标纸按钮点击
    void slotPaperToolButtonClicked();

    //Align表格勾选框状态变化
    void slotAlignTableCheckStateChanged(int index);

    //Ref表格勾选框状态变化
    void slotRefTableCheckStateChanged(int index);

    //切换原点云与目标点云
    void slotCloudSourseChanged(int index);

    //矩阵使用来源改变
    void slotRadioButtonClicked();

    //检查矩阵信息有效性
    bool checkMatrixData();

    //选择保存文件路径
    void slotOpenSavePath();

    //[!]获取选区配准的参数
    void setConstituencyParameter();

    //[!]选区配准的框选操作槽函数
    void slotBoxSelectionOperation(bool);

    //[!]选区配准的框选操作槽函数
    void slotMultipleSelectionOperation(bool);
protected:
    /**
    * @brief 修改按钮图标
    * @param 按钮，图片名字
    * @return
    */
    void setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix);

    /**
    * @brief 读取Excel文件数据
    * @param 文件名，数据
    * @return
    */
    bool getExcelFileData(QString filename, std::vector<CCVector3d> & data, std::vector<QString> & indexNames);


    /**
    * @brief 保存表格数据到excel表格中
    * @param 文件名，数据表
    * @return
    */
    void saveExcelFileData(QString filename, QTableWidget * table);

    /**
    * @brief 读取TXT和CSV文件数据
    * @param 文件名，数据
    * @return
    */
    bool getTxtAndCsvFileData(QString filename, std::vector<CCVector3d> & data, std::vector<QString> & indexNames);

    /**
     * @brief 切换QTableWidget两行数据
     * @param 表，一行，另一行
     * @return
    */
    void SwitchRowData(QTableWidget * table, int uprow, int downrow);
    //! Enables (or not) buttons depending on the number of points in both lists
    void onPointCountChanged();

    //点云拼接函数
    void pointCloudSplicing();

    //Melone.Yang   down sampling pretreatment
    void pretreat(pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_down);

    //Melone.Yang Find overlap
    void getOverlappedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, pcl::PointCloud<pcl::PointXYZ>& overlapped_cloud2);

    //janson 通过特征点，提取邻域点
    void findRadiusClosedCloud(const pcl::PointCloud<pcl::PointXYZ>& indexPoints,
        const pcl::PointCloud<pcl::PointXYZ>& originCloud, pcl::PointCloud<pcl::PointXYZ>& targetCloud);

    //armon.guo 统计滤波去除离群点
    void removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_clean);

    //armon.guo 后处理建图滤波薄化点云
    void fjCloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        int search_nub,
        float dis_p_th,
        float distance_th);

    //! Calls Horn registration (CCCoreLib::HornRegistrationTools)
    bool callHornRegistration(CCCoreLib::PointProjectionTools::Transformation& trans, double& rms, bool autoUpdateTab);

    //! Clears the RMS rows
    void clearRMSColumns();

    //! Adds a point to one of the table (ref./aligned)
    void addPointToTable(QTableWidget* tableWidget, const CCVector3d& P, QString pointName, bool checked);

    //! Converts a picked point to a sphere center (if necessary)
    /** \param P input point (may be converted to a sphere center)
        \param entity associated entity
        \param sphereRadius the detected spherer radius (or -1 if no sphere)
        \return whether the point can be used or not
    **/
    bool convertToSphereCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius);

    /**
  *@brief 标靶纸强度点云中心点提取 janson
  */
    bool convertToIntensityPaperCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius);

    //获取有效坐标点和对应表格indes
    bool getValidPointcloudAndIndex(ccPointCloud& cloudpoints,std::vector<int> & index,QTableWidget * table);

    //判断是否可以转化或配准
    bool getCanAlign();

    //判断当前选中行是否可以删除
    bool getDeleteButtonValid(QTableWidget * table,const std::set<int> & index);

    //更新align标签
    void updateAlignPlontAndLabel();

    //更新表格勾选状态
    void updateTableWidgetCheckState(QTableWidget * table);

    //更新ref标签
    void updateRefPlontAndLabel();

    //加载旧的历史数据
    void loadHistoryData();

    //清空矩阵记录
    void clearMatrixData();

    //更新矩阵信息
    void updateMatrixData();

    //获取界面输入的矩阵
    ccGLMatrixd getGLMatrixData();

    //获取第一个空行
    void getFirstEmptyIndex(int & index, QTableWidget * table);


    //! Entity original context
    struct EntityContext
    {
        //! Default constructor
        explicit EntityContext(ccHObject* ent);

        //! Restores cloud original state
        void restore();

        ccHObject* entity;
        ccGenericGLDisplay* originalDisplay;
        bool wasVisible;
        bool wasEnabled;
        bool wasSelected;
    };

    //! Set of contexts
    struct EntityContexts : public QMap< ccHObject*, EntityContext >
    {
        void fill(const ccHObject::Container& entities);

        void restoreAll()
        {
            for (EntityContext& ctx : *this)
                ctx.restore();
        }

        bool isShifted;
        CCVector3d shift;
        double scale = 1.0;
    };

protected: //members

    //! Aligned entity
    EntityContexts m_alignedEntities;

    //! Aligned points set
    ccPointCloud m_alignedPoints;

    //! Reference entity (if any)
    EntityContexts m_referenceEntities;

    //! Reference points set
    ccPointCloud m_refPoints;

    //! Dedicated window
    ccGLWindow* m_win;

    //! Whether the dialog is paused or not
    bool m_paused;

    //判断提示信息
    bool m_isCancelClickd = false;

    //! Picking hub
    ccPickingHub* m_pickingHub;

    //! Main application interface
    ccMainAppInterface* m_app;

    bool m_isPickOpen = false;

    double m_paperRadius = 0.11;   ///<靶标纸半径

    double m_ballRadius = 0.1;   ///<靶标球半径

    float m_RMSvalue = 30.0;//rms误差百分数

    bool m_isTableDataChangeLock = false;   ///<表格添加行时加锁触发itemChanged

    CurrentPickMode m_currentPickMode = NOPICK;  ///<当前选点模式

    std::vector<HistoryData> m_HistoryDatas;  ///<历史选点数据

    bool m_isAlreadyAlign = false;

    double m_Scalefactor = -1; ///<缩放因子

    std::vector<double> m_TransformationMatrix;  ///<转化矩阵

    CheckBoxHeaderView * m_alignedCheckBoxHeader = nullptr;

    CheckBoxHeaderView * m_refCheckBoxHeader = nullptr;
private:
    std::vector<double>m_icpValueVector;
    constituencyRegistration *m_pSelectionRegistration = nullptr;

    void clearChangeRegistrationType();
};

class icpRegistration : public CS::MetahubWidgets::MetahubFramelessDialog
{
    QOBJECT_H
public:
    explicit icpRegistration(QWidget* parent = nullptr);
    ~icpRegistration();
    
    std::vector<double> getIpcParameters();
    void setIcpParameters(std::vector<double>);
private:
    QCheckBox *m_pOpenCheckBox = nullptr;
    QCheckBox *m_pCloseCheckBox = nullptr;
    QDoubleSpinBox *m_pSpinBox = nullptr;
};
#endif //POINT_PAIR_REGISTRATION_DIALOG_HEADER
