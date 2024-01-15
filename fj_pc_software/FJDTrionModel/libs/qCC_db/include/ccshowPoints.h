

#ifndef __CCSHOWPOINTS_H__
#define __CCSHOWPOINTS_H__

//Local
#include "ccHObject.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

//! 2D label (typically attached to points)
class QCC_DB_LIB_API ccShowPoints : public ccHObject
{
public:
	//! Default constructor
	ccShowPoints(QString name = QString("points"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::OBJECT; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	//清空点
	void clear() { m_points.clear(); }

	//添加点
	void addPoint(const CCVector3 & point) { m_points.push_back(point); }

    //设置点大小
    void setPointSize(unsigned char cloudsize) { m_cloudsize = cloudsize;}

    //设置全局偏移
    void setGlobalShiftPos(CCVector3d point) { m_globalshift = point; }

    void setHeightLightSize(float size) { m_HeightLightPointSize = size; }

    float getHeightLightSize(float size) {return m_HeightLightPointSize; }

    //[!]设置点云是否启动大小自适应
    void setPointSizeAdaptively(bool state = false) {
        m_cloudSizeAdaptively = state;
    }
    //[!]获取点云是否开启自适应大小
    bool getPointSizeAdaptively() { return m_cloudSizeAdaptively; }

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

	std::vector<CCVector3> m_points;

    unsigned char m_cloudsize = 0;

    CCVector3d m_globalshift;

	float m_HeightLightPointSize = 0;

    bool m_cloudSizeAdaptively = false;

};

#endif //CC_2D_LABEL_HEADER
