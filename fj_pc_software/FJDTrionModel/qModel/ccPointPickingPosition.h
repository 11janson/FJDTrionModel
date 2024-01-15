/*
janson.yang
2022.4.11
鼠标选取多个点坐标值

*/

#pragma once
#include "ccPointPickingGenericInterface.h"

using namespace std;

class ccPointCloud;
class ccPickingListener;

class ccPointPickingPosition :
	public ccPointPickingGenericInterface
{
	Q_OBJECT;

public:
	explicit ccPointPickingPosition(ccPickingHub* pickingHub, QWidget* parent);
	~ccPointPickingPosition();

	virtual bool linkWith(ccGLWindow* win) override;

	//关联一个点云或者模型
	//void linkWithEntity(ccHObject* entity);
	void cancelAndExit();

	vector<PickedItem>& getPointList();
protected:
	void processPickedPoint(const PickedItem& picked) override;
	void doProcess();
	void removeLastPick();//撤销上个点

	//关联的点云或者模型
	//ccHObject* m_associatedEntity;
	vector<PickedItem> m_pickPointItems;
	//ccPointCloud* m_cloud;
};

