#include "ccCuboids.h"

//qCC_db
#include "ccPlane.h"
#include "ccPointCloud.h"

#ifdef CC_CORE_LIB_USES_TBB
#include <tbb/tbb.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#include <qdebug.h>

ccCuboids::ccCuboids(const ccRasterGrid& grid,
				QString name/*=QString("Box")*/)
	: ccGenericPrimitive(name)
	, m_grid(grid)
{
	updateRepresentation();
}

ccCuboids::ccCuboids(QString name/*=QString("Box")*/)
	: ccGenericPrimitive(name)
{
}

bool ccCuboids::buildUp()
{
	////clear triangles indexes
	//if (m_triVertIndexes)
	//{
	//	m_triVertIndexes->clear();
	//}
	////clear per triangle normals
	//removePerTriangleNormalIndexes();
	//if (m_triNormals)
	//{
	//	m_triNormals->clear();
	//}
	////clear vertices
	//ccPointCloud* verts = vertices();
	//if (verts)
	//{
	//	verts->clear();
	//}





	////upper plane
	//ccGLMatrix upperMat;
	//upperMat.getTranslation()[2] = m_dims.z / 2;
	//*this += ccPlane(m_dims.x, m_dims.y, &upperMat);
	////lower plane
	//ccGLMatrix lowerMat;
	//lowerMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI), CCVector3(1, 0, 0), CCVector3(0, 0, -m_dims.z / 2));
	//*this += ccPlane(m_dims.x, m_dims.y, &lowerMat);
	////left plane
	//ccGLMatrix leftMat;
	//leftMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(-m_dims.x / 2, 0, 0));
	//*this += ccPlane(m_dims.z, m_dims.y, &leftMat);
	////right plane
	//ccGLMatrix rightMat;
	//rightMat.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(m_dims.x / 2, 0, 0));
	//*this += ccPlane(m_dims.z, m_dims.y, &rightMat);
	////front plane
	//ccGLMatrix frontMat;
	//frontMat.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, -m_dims.y / 2, 0));
	//*this += ccPlane(m_dims.x, m_dims.z, &frontMat);
	////back plane
	//ccGLMatrix backMat;
	//backMat.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, m_dims.y / 2, 0));
	//*this += ccPlane(m_dims.x, m_dims.z, &backMat);

	return (vertices() && vertices()->size() == 24 && this->size() == 12);
}

ccGenericPrimitive* ccCuboids::clone() const
{
	return finishCloneJob(new ccCuboids(m_grid, getName()));
}

bool ccCuboids::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	////parameters (dataVersion>=21)
	//QDataStream outStream(&out);
	//outStream << m_dims.x;
	//outStream << m_dims.y;
	//outStream << m_dims.z;

	return true;
}

bool ccCuboids::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	////parameters (dataVersion>=21)
	//QDataStream inStream(&in);
	//ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_dims.u, 3);

	return true;
}

void ccCuboids::convertGrid2Cuboids(const std::vector<ccRasterGrid::ExportableFields>& exportedFields, unsigned char Z, bool fillEmptyCells)
{
    if (Z > 2)
    {
        //invalid input parameters
        assert(false);
        return;
    }

    unsigned width = m_grid.width;
    unsigned height = m_grid.height;
    CCVector3d minCorner = m_grid.minCorner;
    std::vector<ccRasterGrid::Row>& rows = m_grid.rows;
    double gridStep = m_grid.gridStep;
    unsigned validCellCount = m_grid.validCellCount;

    unsigned pointsCount = (fillEmptyCells ? width * height : validCellCount);
    if (pointsCount == 0)
    {
        ccLog::Warning("[Rasterize] Empty grid!");
        return;
    }

    //6个平面，12个三角面
    //初始化8个顶点,三角面,法向量 
    init(pointsCount * 8, false, pointsCount * 12, pointsCount * 6);
    ccPointCloud* verts = vertices();

    //shall we generate additional scalar fields?
    std::vector<CCCoreLib::ScalarField*> exportedSFs;
    if (!exportedFields.empty())
    {
        exportedSFs.resize(exportedFields.size(), nullptr);
        for (size_t i = 0; i < exportedFields.size(); ++i)
        {
            int sfIndex = -1;
            switch (exportedFields[i])
            {
            case ccRasterGrid::PER_CELL_HEIGHT:
            case ccRasterGrid::PER_CELL_COUNT:
            case ccRasterGrid::PER_CELL_MIN_HEIGHT:
            case ccRasterGrid::PER_CELL_MAX_HEIGHT:
            case ccRasterGrid::PER_CELL_AVG_HEIGHT:
            case ccRasterGrid::PER_CELL_HEIGHT_STD_DEV:
            case ccRasterGrid::PER_CELL_HEIGHT_RANGE:
            {
                QString sfName = ccRasterGrid::GetDefaultFieldName(exportedFields[i]);
                sfIndex = verts->getScalarFieldIndexByName(qPrintable(sfName));
                if (sfIndex >= 0)
                {
                    ccLog::Warning(QString("[Rasterize] Scalar field '%1' already exists. It will be overwritten.").arg(sfName));
                }
                else
                {
                    sfIndex = verts->addScalarField(qPrintable(sfName));
                }
            }
            break;
            default:
                assert(false);
                break;
            }

            if (sfIndex < 0)
            {
                ccLog::Warning("[Rasterize] Couldn't allocate scalar field(s)! Try to free some memory ...");
                break;
            }

            exportedSFs[i] = verts->getScalarField(sfIndex);
            assert(exportedSFs[i]);
        }
    }


    

    //horizontal dimensions
    const unsigned char X = (Z == 2 ? 0 : Z + 1);
    const unsigned char Y = (X == 2 ? 0 : X + 1);

    bool exportToOriginalCS = true;
    const unsigned char outX = (exportToOriginalCS ? X : 0);
    const unsigned char outY = (exportToOriginalCS ? Y : 1);
    const unsigned char outZ = (exportToOriginalCS ? Z : 2);

    //as the 'non empty cells points' are already in the cloud
    //we must take care of where we put the scalar fields values!
    unsigned nonEmptyCellIndex = 0;

    //we work with doubles as the grid step can be much smaller than the cloud coordinates!
    double min_y = minCorner.u[Y]; //minCorner is the lower left cell CENTER

    qDebug() << "START---convertGrid2Cuboids********";

    
    CCVector3 A, B, C, D, E, F, G, H;//长方体八个顶点
    //float ratio = 0.95;
    float ratio = 1.0;
    float maxHeight, minHeight;
    for (unsigned j = 0; j < height; ++j)
    {
        double Py = j * gridStep + min_y - 0.5*gridStep;

        std::vector<ccRasterCell>::iterator aCell = rows[j].begin();
        double min_x = minCorner.u[X]; //minCorner is the lower left cell CENTER
        //double Px = 0 * gridStep + min_x;
        for (unsigned i = 0; i < width; ++i, ++aCell)
        {
            double Px = i * gridStep + min_x - 0.5*gridStep;

            if (std::isfinite(aCell->minHeight) && std::isfinite(aCell->maxHeight))
            {
                if (aCell->maxHeight > aCell->minHeight)
                {
                    maxHeight = aCell->maxHeight;
                    minHeight = aCell->minHeight;
                }
                else
                {
                    minHeight = aCell->maxHeight;
                    maxHeight = aCell->minHeight;
                }

                {
                    if (Z == 2)
                    {
                        A = CCVector3(Px, Py, maxHeight);
                        B = CCVector3(Px + ratio * gridStep, Py, maxHeight);
                        C = CCVector3(Px + ratio * gridStep, Py + ratio * gridStep, maxHeight);
                        D = CCVector3(Px, Py + ratio * gridStep, maxHeight);

                        E = CCVector3(Px, Py, minHeight);
                        F = CCVector3(Px + ratio * gridStep, Py, minHeight);
                        G = CCVector3(Px + ratio * gridStep, Py + ratio * gridStep, minHeight);
                        H = CCVector3(Px, Py + ratio * gridStep, minHeight);
                    }
                    else if (Z == 1)
                    {
                        A = CCVector3(Py, minHeight, Px + ratio * gridStep);
                        B = CCVector3(Py + ratio * gridStep, minHeight, Px + ratio * gridStep);
                        C = CCVector3(Py + ratio * gridStep, maxHeight, Px + ratio * gridStep);
                        D = CCVector3(Py, maxHeight, Px + ratio * gridStep);

                        E = CCVector3(Py, minHeight, Px);
                        F = CCVector3(Py + ratio * gridStep, minHeight, Px);
                        G = CCVector3(Py + ratio * gridStep, maxHeight, Px);
                        H = CCVector3(Py, maxHeight, Px);
                    }
                    else if (Z == 0)
                    {
                        A = CCVector3(minHeight, Px, Py + ratio * gridStep);
                        B = CCVector3(maxHeight, Px, Py + ratio * gridStep);
                        C = CCVector3(maxHeight, Px + ratio * gridStep, Py + ratio * gridStep);
                        D = CCVector3(minHeight, Px + ratio * gridStep, Py + ratio * gridStep);

                        E = CCVector3(minHeight, Px, Py);
                        F = CCVector3(maxHeight, Px, Py);
                        G = CCVector3(maxHeight, Px + ratio * gridStep, Py);
                        H = CCVector3(minHeight, Px + ratio * gridStep, Py);
                    }
                    else
                        return;

                    //八个顶点
                    int count = verts->size();
                    verts->addPoint(A);
                    verts->addPoint(B);
                    verts->addPoint(C);
                    verts->addPoint(D);
                    verts->addPoint(E);
                    verts->addPoint(F);
                    verts->addPoint(G);
                    verts->addPoint(H);

                    int normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1)));//法向量
                    addTriangle(count + 0, count + 1, count + 2);//两个三角面片，顺时针和逆时针代表正反面
                    addTriangle(count + 0, count + 2, count + 3);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);//三角面片三个顶点的法向量索引
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);

                    normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, -1)));
                    addTriangle(count+4, count+7, count+6);
                    addTriangle(count+4, count+6, count+5);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);

                    normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, -1, 0)));
                    addTriangle(count+0, count+4, count+1);
                    addTriangle(count+4, count+5, count+1);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);

                    normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 1, 0)));
                    addTriangle(count+7, count+2, count+6);
                    addTriangle(count+7, count+3, count+2);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);

                    normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(1, 0, 0)));
                    addTriangle(count+0, count+4, count+3);
                    addTriangle(count+4, count+7, count+3);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);

                    normalCount = m_triNormals->currentSize();
                    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(-1, 0, 0)));
                    addTriangle(count+1, count+2, count+5);
                    addTriangle(count+5, count+2, count+6);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);
                    addTriangleNormalIndexes(normalCount, normalCount, normalCount);


                    //CCVector3 center, m_dims;
                    //center.u[outX] = static_cast<PointCoordinateType>(Px + 0.5*gridStep);
                    //center.u[outY] = static_cast<PointCoordinateType>(Py + 0.5*gridStep);
                    //center.u[outZ] = static_cast<PointCoordinateType>(0.5*(aCell->minHeight + aCell->maxHeight));

                    //m_dims.u[outX] = 0.95*gridStep;
                    //m_dims.u[outY] = 0.95*gridStep;
                    //m_dims.u[outZ] = aCell->h;



                    ////upper plane
                    ////ccGLMatrix upperMat;
                    //tempMatrix.initFromParameters(static_cast<PointCoordinateType>(0), CCVector3(1, 0, 0), CCVector3(0, 0, m_dims.z / 2) + center);
                    //*this += ccPlane(m_dims.x, m_dims.y, &tempMatrix);
                    ////lower plane
                    ////ccGLMatrix lowerMat;
                    //tempMatrix.initFromParameters(-static_cast<PointCoordinateType>(M_PI), CCVector3(1, 0, 0), CCVector3(0, 0, -m_dims.z / 2) + center);
                    //*this += ccPlane(m_dims.x, m_dims.y, &tempMatrix);
                    ////left plane
                    ////ccGLMatrix leftMat;
                    //tempMatrix.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(-m_dims.x / 2, 0, 0) + center);
                    //*this += ccPlane(m_dims.z, m_dims.y, &tempMatrix);
                    ////right plane
                    ////ccGLMatrix rightMat;
                    //tempMatrix.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(0, 1, 0), CCVector3(m_dims.x / 2, 0, 0) + center);
                    //*this += ccPlane(m_dims.z, m_dims.y, &tempMatrix);
                    ////front plane
                    ////ccGLMatrix frontMat;
                    //tempMatrix.initFromParameters(static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, -m_dims.y / 2, 0) + center);
                    //*this += ccPlane(m_dims.x, m_dims.z, &tempMatrix);
                    ////back plane
                    ////ccGLMatrix backMat;
                    //tempMatrix.initFromParameters(-static_cast<PointCoordinateType>(M_PI / 2), CCVector3(1, 0, 0), CCVector3(0, m_dims.y / 2, 0) + center);
                    //*this += ccPlane(m_dims.x, m_dims.z, &tempMatrix);


                    //fill the associated SFs
                    assert(exportedSFs.size() == exportedFields.size());
                    for (size_t k = 0; k < exportedSFs.size(); ++k)
                    {
                        CCCoreLib::ScalarField* sf = exportedSFs[k];
                        if (!sf)
                        {
                            continue;
                        }

                        ScalarType sVal = CCCoreLib::NAN_VALUE;
                        switch (exportedFields[k])
                        {
                        case ccRasterGrid::PER_CELL_HEIGHT:
                            sVal = static_cast<ScalarType>(aCell->h);
                            break;
                        case ccRasterGrid::PER_CELL_COUNT:
                            sVal = static_cast<ScalarType>(aCell->nbPoints);
                            break;
                        case ccRasterGrid::PER_CELL_MIN_HEIGHT:
                            sVal = static_cast<ScalarType>(aCell->minHeight);
                            break;
                        case ccRasterGrid::PER_CELL_MAX_HEIGHT:
                            sVal = static_cast<ScalarType>(aCell->maxHeight);
                            break;
                        case ccRasterGrid::PER_CELL_AVG_HEIGHT:
                            sVal = static_cast<ScalarType>(aCell->avgHeight);
                            break;
                        case ccRasterGrid::PER_CELL_HEIGHT_STD_DEV:
                            sVal = static_cast<ScalarType>(aCell->stdDevHeight);
                            break;
                        case ccRasterGrid::PER_CELL_HEIGHT_RANGE:
                            sVal = static_cast<ScalarType>(aCell->maxHeight - aCell->minHeight);
                            break;
                        default:
                            assert(false);
                            break;
                        }

                        if (0)
                        {
                            //if (aCell->nbPoints != 0)
                            //{
                            //    //previously existing point
                            //    assert(nonEmptyCellIndex < inputCloud->size());
                            //    assert(nonEmptyCellIndex < sf->size());
                            //    sf->setValue(nonEmptyCellIndex, sVal);
                            //}
                            //else
                            //{
                            //    //new point
                            //    assert(sf->size() < sf->capacity());
                            //    sf->addElement(sVal);
                            //}
                        }
                        else
                        {
                            //new point
                            assert(sf->size() < sf->capacity());
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                            sf->addElement(sVal);
                        }
                    }




                }
            }
            else if (fillEmptyCells) //empty cell
            {
                ////even if we have resampled the original cloud, we must add the point
                ////corresponding to this empty cell
                //{
                //    CCVector3 Pf;
                //    Pf.u[outX] = static_cast<PointCoordinateType>(Px);
                //    Pf.u[outY] = static_cast<PointCoordinateType>(Py);
                //    Pf.u[outZ] = static_cast<PointCoordinateType>(emptyCellsHeight);

                //    assert(cloudGrid->size() < cloudGrid->capacity());
                //    cloudGrid->addPoint(Pf);

                //    if (interpolateColors)
                //    {
                //        cloudGrid->addColor(ccColor::black);
                //    }
                //}

                //assert(exportedSFs.size() == exportedFields.size());
                //for (size_t k = 0; k < exportedSFs.size(); ++k)
                //{
                //    CCCoreLib::ScalarField* sf = exportedSFs[k];
                //    if (!sf)
                //    {
                //        continue;
                //    }

                //    if (exportedFields[k] == PER_CELL_HEIGHT)
                //    {
                //        //we set the point height to the default height
                //        ScalarType s = static_cast<ScalarType>(emptyCellsHeight);
                //        assert(sf->size() < sf->capacity());
                //        sf->addElement(s);
                //    }
                //    else
                //    {
                //        assert(sf->size() < sf->capacity());
                //        sf->addElement(CCCoreLib::NAN_VALUE);
                //    }
                //}
            }

            //Px += gridStep;
        }

        //Py += gridStep;
    }

    verts->shrinkToFit();

    qDebug() << "FINISH---convertGrid2Cuboids********";

    return;
}

//const ccGenericPrimitive& ccCuboids::operator += (const ccGenericPrimitive& prim)
//{
//    ccPointCloud* verts = vertices();
//    unsigned vertCount = verts->size();
//    unsigned facesCount = size();
//    unsigned triFacesNormCount = (m_triNormals ? m_triNormals->currentSize() : 0);
//
//    //count new number of vertices & faces
//    unsigned newVertCount = vertCount + prim.getAssociatedCloud()->size();
//    unsigned newFacesCount = facesCount + prim.size();
//    bool primHasVertNorms = prim.getAssociatedCloud()->hasNormals();
//    bool primHasFaceNorms = prim.hasTriNormals();
//    bool primHasColors = prim.getAssociatedCloud()->hasColors();
//
//    //reserve memory
//    if (verts->reserve(newVertCount)
//        /*&& (!primHasVertNorms || verts->reserveTheNormsTable())
//        && reserve(newFacesCount)
//        && (!primHasFaceNorms || m_triNormalIndexes || reservePerTriangleNormalIndexes())*/)
//    {
//
//        //copy vertices & normals & colors
//        ccGenericPointCloud* cloud = prim.getAssociatedCloud();
//        for (unsigned i = 0; i < cloud->size(); ++i)
//        {
//            verts->addPoint(*cloud->getPoint(i));
//        }
//
//       
//
//        //copy faces
//        for (unsigned i = 0; i < prim.size(); ++i)
//        {
//            const CCCoreLib::VerticesIndexes* tsi = prim.getTriangleVertIndexes(i);
//            addTriangle(vertCount + tsi->i1, vertCount + tsi->i2, vertCount + tsi->i3);     
//        }
//    }
//    else
//    {
//        ccLog::Error("[ccGenericPrimitive::operator +] Not enough memory!");
//    }
//
//    return *this;
//}

