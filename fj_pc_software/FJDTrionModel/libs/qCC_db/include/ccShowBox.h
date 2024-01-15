//##########################################################################
//显示包围盒实体，用来保存长方体体积测量的结果
//janson.yang 2022.6.6
//##########################################################################

#ifndef CC_SHOW_BOX_HEADER
#define CC_SHOW_BOX_HEADER

//Local
#include "ccBBox.h"
#include "ccHObject.h"

//Qt
#include <QObject>

class QCC_DB_LIB_API ccShowBox : public ccHObject
{

public:

	//! Default constructor
	ccShowBox(QString name= QString("show box"));

	//! Destructor
	~ccShowBox() override;

	//! Returns unique class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SHOW_BOX; }//在dbtree中前面，出现可选框
	virtual ccBBox getOwnBB(bool withFeatures) override { return m_box; }//使show name in 3d可用

	//! Sets the box extents
	void setBox(const ccBBox& box);
	void setBBoxWidthColor(float width, ccColor::Rgba col) { m_box.setWidth(width);  box_color = col; }
protected: //methods

	//inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

protected: //members

	//! Clipping box
	ccBBox m_box;
	ccColor::Rgba box_color;

};

#endif //CC_SHOW_BOX_HEADER
