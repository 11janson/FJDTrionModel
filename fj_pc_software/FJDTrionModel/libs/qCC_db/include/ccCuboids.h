/******************************************************************************
@brief : 可视化立方体网格头文件，借鉴ccBox类
@author : janson.yang
@date : 2022/8/4
@version : ver 1.0
*****************************************************************************/

#ifndef CC_CUBOIDS_PRIMITIVE_HEADER
#define CC_CUBOIDS_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"
#include "ccRasterGrid.h"


//! Box (primitive)
/** 3D box primitive
**/
class QCC_DB_LIB_API ccCuboids : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Box dimensions axis along each dimension are defined in a single 3D vector.
		A box is in fact composed of 6 planes (ccPlane).
		\param dims box dimensions
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccCuboids(const ccRasterGrid& grid,
			QString name = QString("Cublids"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCuboids(QString name = QString("Cuboids"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CUBOIDS; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Cuboids"; }
	virtual ccGenericPrimitive* clone() const override;

    //const ccGenericPrimitive& operator += (const ccGenericPrimitive& prim);

    void convertGrid2Cuboids(const std::vector<ccRasterGrid::ExportableFields>& exportedFields, unsigned char Z, bool fillEmptyCells);
protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual bool buildUp() override;

    ccRasterGrid m_grid;
};

#endif //CC_CUBOIDS_PRIMITIVE_HEADER
