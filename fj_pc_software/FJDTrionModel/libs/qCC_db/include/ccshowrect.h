#ifndef __CCSHOWRECT_H__
#define __CCSHOWRECT_H__

//Local
#include "ccHObject.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

//! 2D label (typically attached to points)
class QCC_DB_LIB_API ccShowRect : public ccHObject
{
public:
	//! Default constructor
	ccShowRect(QString name = QString("line"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::OBJECT; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	void setPointData(std::vector<CCVector3d> param);
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
	std::vector<CCVector3d> m_PointParam;
};

#endif //CC_2D_LABEL_HEADER
