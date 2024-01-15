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

#ifndef __CCPOINTACCURACYVERIFICATIONDLG_H__
#define __CCPOINTACCURACYVERIFICATIONDLG_H__

//Local
#include "ccMainAppInterface.h"
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"
#include "metahubtoolbutton.h"

//CCCoreLib
#include <PointProjectionTools.h>

//qCC_db
#include <ccPointCloud.h>

//Qt generated dialog
#include <ui_pointAccuracyVerificationDlg.h>

//PCL
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define MAXDELTA 999999
#define ISNULL NULL

class ccGenericPointCloud;
class ccGenericGLDisplay;
class ccGLWindow;
class cc2DLabel;
class ccPickingHub;

//Dialog for the point-pair registration algorithm (Horn)
class ccPointAccuracyVerificationDlg : public ccOverlayDialog, public ccPickingListener, Ui::pointAccuracyVerificationDlg
{
	Q_OBJECT

public:

	enum CurrentCompareType {
		ALTITUDE = 0,  //�߳�
		ALTITUDEANDPLANE //�߳�ƽ��
	};

	//��������
	struct PointPositionAccuracyType
	{
		//������֤����
		float MeanSquareError;
		float averageError;
		float maxValue;
		float minValue;
		double pointCloudZCoord;
		//��λ��������
		float deltaX;
		float deltaY;
		float deltaL;
		float deltaZ;

	};

	struct ElevationVerify
	{
		//�߳���֤
		float posx;
		float posy;
		float posz;
		//
		float controlx;
		float controly;
		float controlz;
	};
	struct PlaneVerification
	{
		//ƽ����֤
		float posx;
		float posy;
		float posz;
		//
		float controlx;
		float controly;
	};
	struct Parameters
	{
		//parameters
		double zTolerance;
		double matchingNeighborhood;
	}; Parameters params;

	//shangle ���4��vector�������㷨���������
	std::vector<PointPositionAccuracyType> precisionDelta;
	std::vector<PointPositionAccuracyType> precisionAverageDelta;
	std::vector<PointPositionAccuracyType> planeDelta;
	std::vector<PointPositionAccuracyType> planeAverageDelta;
	//����tablewidget�������
	std::vector<CCVector3d>cloudpos;
	std::vector<CCVector3d>controlpos;
	//����·��
	QString Path;

	bool computeHeightAndPlaneAccuracy(std::vector<CCVector3> pickPoint, std::vector<CCVector3> controlPoint);

	bool computeHeightAccuracy(std::vector<CCVector3> controlPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/**
	*@brief ��ȡ�Ա�����
	*/
	CurrentCompareType getCurrentCompareType();

	/**
	*@brief ��ȡZ�ݲ�
	*/
	double getCurrentZTolerance();

	/**
	*@brief ��ȡƥ������
	*/
	double getMatchingNeighborhood();

	/**
	*@brief ��ȡ���Ƶ�
	*/
	ccPointCloud getControlPoint();

	/**
	*@brief ��ȡʰȡ��
	*/
	ccPointCloud getPickPoints();
	/**
	*@brief ��ȡ��������
	*/
	void getCloudPos();
	/**
	 *@brief ��ȡ���Ƶ�����
	*/
	void  getControlPos();

	//! Default constructor
	explicit ccPointAccuracyVerificationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent = nullptr);

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
	bool addAlignedPoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true);
	//! Adds a point to the 'reference' set
	bool addReferencePoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true);

	//! Removes a point from the 'align' set
	void removeAlignedPoint(int index, bool autoRemoveDualPoint = true);
	//! Removes a point from the 'reference' set
	void removeRefPoint(int index, bool autoRemoveDualPoint = true);

	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

	void SwitchRowData(QTableWidget * table, int uprow, int downrow);

signals:

	void showMessage(QString mes);

	void openPutDialog();
    void updatedialogUIHeight(int);

public slots:

    void updateDialogUi(int);
	//! Slot called to change aligned entities visibility
	void showAlignedEntities(bool);
	//! Slot called to change reference entities visibility
	void showReferenceEntities(bool);

	//! Slot called to remove the last point on the 'align' stack
	void unstackAligned();
	//! Slot called to remove the last point on the 'reference' stack
	void unstackRef();


	//! Updates the registration info and buttons states
	void updateAlignInfo();

	void apply();
	void cancel();

	/**
	* @brief ��ȡTXT��CSV�ļ�����
	* @param �ļ���������
	* @return
	*/
	bool getTxtFileData(QString filename, std::vector<CCVector3d> & data);


	/**
	* @brief ��ȡExcel�ļ�����
	* @param �ļ���������
	* @return
	*/
	bool getExcelFileData(QString filename, std::vector<CCVector3d> & data);

	/**
    * @brief ��ȡCsv�ļ�����
    * @param �ļ���������
    * @return
    */
	bool getCsvFileData(const QString& filename, std::vector<CCVector3d>& data);

	bool isValidDouble(const QString& str);
	void upSource();
	void downSource();
	void deleteSource();
	void upTarget();
	void downTarget();
	void deleteTarget();
    void addSource();
    void addTarget();
	void setAccuracyInformation();
	void updateTypeChanged();

protected:

	/**
	* @brief �޸İ�ťͼ��
	* @param ��ť��ͼƬ����
	* @return
	*/
	void setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix);
	void setIsDownPartHide(bool ishide);

	//! Enables (or not) buttons depending on the number of points in both lists
	void onPointCountChanged();


	//! Calls Horn registration (CCCoreLib::HornRegistrationTools)
	bool callHornRegistration(CCCoreLib::PointProjectionTools::Transformation& trans, double& rms, bool autoUpdateTab);

	//! Clears the RMS rows
	void clearRMSColumns();

	//! Adds a point to one of the table (ref./aligned)
	void addPointToTable(QTableWidget* tableWidget,
		int rowIndex,
		const CCVector3d& P,
		QString pointLabel);



	//! Resets the displayed title (3D view)
	void resetTitle();

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

	//! Removes a label (and restore its associated label if any)
	void removeLabel(ccPointCloud& points,
		unsigned childIndex,
		const EntityContexts& entities);

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

	//! Picking hub
	ccPickingHub* m_pickingHub;

	//! Main application interface
	ccMainAppInterface* m_app;


	float m_RMSvalue = 30.0;//rms���ٷ���

	CurrentCompareType m_CurrentCompareType = ALTITUDE;  //��ǰ��֤ģʽ

	bool m_isPickModeOpen = true;   ///<ʰȡ��ť�Ƿ���

	bool m_isTableDataChangeLock = false;   ///<��������ʱ��������itemChanged



};

#endif //POINT_PAIR_REGISTRATION_DIALOG_HEADER
