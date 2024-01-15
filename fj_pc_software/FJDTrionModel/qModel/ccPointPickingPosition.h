/*
janson.yang
2022.4.11
���ѡȡ���������ֵ

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

	//����һ�����ƻ���ģ��
	//void linkWithEntity(ccHObject* entity);
	void cancelAndExit();

	vector<PickedItem>& getPointList();
protected:
	void processPickedPoint(const PickedItem& picked) override;
	void doProcess();
	void removeLastPick();//�����ϸ���

	//�����ĵ��ƻ���ģ��
	//ccHObject* m_associatedEntity;
	vector<PickedItem> m_pickPointItems;
	//ccPointCloud* m_cloud;
};

