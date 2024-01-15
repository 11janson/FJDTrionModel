//##########################################################################
//��ʾ��Χ��ʵ�壬�������泤������������Ľ��
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
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SHOW_BOX; }//��dbtree��ǰ�棬���ֿ�ѡ��
	virtual ccBBox getOwnBB(bool withFeatures) override { return m_box; }//ʹshow name in 3d����

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
