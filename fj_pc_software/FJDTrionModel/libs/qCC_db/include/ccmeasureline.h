

#ifndef __CCMEASURELINE_H__
#define __CCMEASURELINE_H__

//Local
#include "ccHObject.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

//点云测量显示辅助类
class QCC_DB_LIB_API ccMeasureLine : public ccHObject
{
public:
	enum MeasureLineMode
	{
		MeasureLine_DISTANCE = 0,
		MeasureLine_ANGLE = 1,
		MeasureLine_HEIGHT = 2,
		MeasureLine_AERA = 3,
		MeasureLine_WITHOUTENDPOINTS =4,
	};
	//! Default constructor
	ccMeasureLine(QString name = QString("line"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::OBJECT; }

	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	//设置测量选点
	void setCurrentPos(const std::vector<CCVector3d> & points) { m_pointData = points;}

	//获取测量选点
	std::vector<CCVector3d> getCurrentPos() { return m_pointData;}

	//清空测量选点
	void clearCurrentPos() { m_pointData.clear(); m_areaPoint2dList.clear(); }

	//添加测量选点
	void addCurrentPos(const CCVector3d &point) { m_pointData.push_back(point); }

	//修改最后一个测量点
	void setLastCurrentPos(const CCVector3d &point);

	//设置鼠标移动时是否显示测量标签
	void setLabelVisiable(bool isShow); 

	//设置当前测量模式
	void setCurrentMeasureMode(MeasureLineMode mode) { m_measureMode = mode; }

	//设置测量点云是否为3D点云
	void setIs3dPointMode(bool is3d) { m_is3dPointMode = is3d; }

	//设置面积测量结果
	void setMeasureAreaStr(QString str) { m_areaStr = str; }

    //设置2D坐标点
    void set2dPoint(const std::vector<CCVector3d> & points) { m_areaPoint2dList = points; }

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
	QColor m_color = QColor(Qt::red);  //当前绘制颜色

	std::vector<CCVector3d> m_pointData;  //当前除鼠标以外的点

	bool m_isShowLabel = false;  //是否显示测量结果

	MeasureLineMode m_measureMode = MeasureLine_HEIGHT;  //当前测量模式

	bool m_is3dPointMode = true;   //测量点云是否为3D点云

	QString m_areaStr;

	std::vector<CCVector3d> m_areaPoint2dList;  //测量面积选点2D
};

#endif //CC_2D_LABEL_HEADER
