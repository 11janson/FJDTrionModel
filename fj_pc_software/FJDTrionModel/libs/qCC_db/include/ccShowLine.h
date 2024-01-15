

#ifndef __CCSHOWLINE_H__
#define __CCSHOWLINE_H__

//Local
#include "ccHObject.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

//! 2D label (typically attached to points)
class QCC_DB_LIB_API ccShowLine : public ccHObject
{
public:
	//! Default constructor
	ccShowLine(QString name = QString("line"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::OBJECT; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	void setMousePos(const CCVector3d & mousePoint);

	void setFirstPos(const CCVector3d & point);

	void setMousePosVisiable(bool isShow);

    //设置二维坐标点
    void set2DpointList(const std::vector<CCVector3d> & pointList) {m_point2dList = pointList;}

    //获取3D坐标点
    std::vector<CCVector3d> get3DPointLists();

    void setPointSize(float size);

protected:


	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	virtual void onDeletionOf(const ccHObject* obj) override;

	//! Draws the entity only (not its children) - 2D version
	void drawMeOnly2D(CC_DRAW_CONTEXT& context);
	//! Draws the entity only (not its children) - 3D version
	void drawMeOnly3D(CC_DRAW_CONTEXT& context);

private:
	QColor m_color = QColor(Qt::red);
	CCVector3d m_startPoint = CCVector3d(0, 0, 0);
	CCVector3d m_mousePoint = CCVector3d(0,0,0);
	bool m_isShowMousePos = false;
    std::vector<CCVector3d> m_point2dList;  //选点2D
    float m_pointsize = 5.0f;
};

#endif //CC_2D_LABEL_HEADER
