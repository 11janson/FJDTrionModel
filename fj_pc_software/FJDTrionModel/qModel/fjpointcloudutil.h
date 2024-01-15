#ifndef __FJPOINTCLOUDUTIL_H__
#define __FJPOINTCLOUDUTIL_H__
#include "ccObject.h"
#include <QSharedPointer>
#include "ccColorScalesManager.h"
#include <QString>
#include <QToolButton>
#include <QLineEdit>

struct ClassficationData
{
	ClassficationData(int iid, QString iname, bool iisShow, QColor icolor) { id = iid; name = iname; isShow = iisShow; color = icolor; };
	int id;
	QString name = "";
	bool isShow = true;
	QColor color;
};

class ccHObject;
class QColor;
class ccColorScale;
///////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	点云应用通用方法类. </summary>
///
/// <remarks>	FJPointCloudUtil, 2022/9/28. </remarks>
///////////////////////////////////////////////////////////////////////////////////////////////////

class FJPointCloudUtil
{
public:
	///<summary> 更新并获取点云渲染状态. </summary>
	static CurrentCombinationMode getUpdateCurrentCombinationMode(ccHObject * entity);

	///<summary> 点云组合渲染,根据组合的sf的index. </summary>
	static void pointCloudCompositeRendering(ccHObject * entity, int sf1Idx, int sf2Idx);

    //aric.tang_2022.11.21_high as the status for color scale
    ///<summary> 点云组合渲染,根据组合的sf的index(High). </summary>
    static void pointCloudCompositeRenderingHigh(ccHObject * entity, int sf1Idx, int sf2Idx);

	///<summary> 点云组合渲染,根据组合的sf的名称. </summary>
	static void pointCloudCompositeRendering(ccHObject * entity, const QString & sf1Name, const QString &  sf2Name);

	///<summary> 根据sf名称获取index. </summary>
	static int getSFIndexFromEntity(ccHObject * entity, const QString & sfName);

	///<summary> 设置单色. </summary>
	static void setEntityUniqueColor(ccHObject * ent,QColor colour);

	///<summary> 设置颜色条. </summary>
	static void setEntityColorScale(ccHObject * ent, ccColorScalesManager::DEFAULT_SCALES scaleType);

	///<summary> 导出高程标量. </summary>
	static void exportAltitudeColorScale(ccHObject * ent);

	///<summary> 更新高程标量. </summary>
	static void updateAltitudeColorScale(ccHObject * ent);

	///<summary> 设置不按照标量渲染. </summary>
	static void setNotShowColorScale(ccHObject * ent);

	/** brief create Combinate scalar fields Scale
	  * param[out] scale the color scale for two scalar fields combin
	  * param[in] scaleType color scale num
	  * param[in] unique_color unique color you choosed
	*/
	static QSharedPointer<ccColorScale> createCombinScale(int scaleType, QColor unique_color);


	/** brief measure classification combin with others
	 * param[out] entity choosing entity
	 * param[in] sf1Name first scalar field
	 * param[in] sf2Name second scalar field
	*/
	static void Classification_combin(ccHObject * entity, const QString & sf1Name, const QString &  sf2Name);

	/** brief create classification Scale combin with others
	 * param[out] scale the color scale for classification combin with others
	 * param[in] scaleType color scale num
	 * param[in] class_map class color map
	*/
	static QSharedPointer<ccColorScale> createClassificationScale(int scaleType, std::map<int, QColor> class_map);

    //aric.tang_2022.11.21
    /** brief create High Scale combin with others
     * param[out] scale the color scale for High combin with others
     * param[in] scaleType color scale num
     * param[in] class_map class color map
    */
    static QSharedPointer<ccColorScale> createHighScale(int scaleType, std::vector<QColor> high_map);

	///<summary> 从配置文件读取类别标量颜色配置. </summary>
	static std::vector<ClassficationData> getClassficationDataFromDoc();

	///<summary> 获取类别标量默认颜色配置. </summary>
	static std::vector<ClassficationData> getInitClassficationData();

	///<summary> 根据ID获取类别颜色. </summary>
	static QColor getColorByClassficationID(int id);

	///<summary> 根据ID设置类别颜色. </summary>
	static void setColorByClassficationID(int id, QColor color);

	///<summary> 根据ID设置类别名称. </summary>
	static void setNameByClassficationID(int id, QString name);

	///<summary> 向配置文件保存类别标量颜色配置. </summary>
	static void saveClassficationDataToDoc(const std::vector<ClassficationData> & data);

	///<summary> 根据配置文件保存类别标量颜色配置更新颜色条. </summary>
	static void updateClassficationDataFromDoc(ccHObject * entity,bool isChangesf = true);

	///<summary> 根据配置文件保存类别标量颜色配置更新所有点云颜色条. </summary>
	static void updateAllPointCloudClassficationDataFromDoc();

	///<summary> 点云是否支持色带条设置. </summary>
	static bool isCloudPointSupportScaleSetting(ccHObject * ent);

    ///<summary> 根据阈值设置标量饱和度. </summary>
    static void updateSaturationByRange(ccHObject * ent, double start, double stop);

	///<summary> 点云是否包含该sf. </summary>
	static bool isCloudHasSF(ccHObject * ent,QString name);

	///<summary> 点云当前sf是否为name. </summary>
	static bool isCloudIsCurrentSF(ccHObject * ent, QString name);

	///<summary> 点云设置当前sf为name. </summary>
	static void setCloudSF(ccHObject * ent, QString name);

	///<summary> 更新标签名字. </summary>
	static void updateLabelName(ccHObject * ent);

	///<summary> 修改toolbutton图标. </summary>
	static void setActionIcon(QToolButton * button, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled);

	///<summary> 获取类别颜色条. </summary>
	static QSharedPointer<ccColorScale> getClassificationScale(ccHObject * entity);

	///<summary> 获取颜色卡的颜色. </summary>
	static QColor getQColorDialogColor(const QString & title,const QColor & origoncolor,bool & isvalid);

    ///<summary> 根据项目树上下关系排序. </summary>
    static void sortEntityByTree(std::vector<ccHObject *> & entitys);

    ///<summary> 获取项目树上节点顺序. </summary>
    static std::vector<int> getEntityIndexFromTree(ccHObject * entity);

    ///<summary> 判断目录是否具有写入权限,第二个参数是否显示提示框</summary>
    static bool getDirWriteable(QString path,bool isDisplayMessbox = true);

	//[!]参数:infodata的文件夹路径,一个文件夹里包含两个infodata时会返回false
    ///第二个参数：生成矩阵文件
    static bool beginParseInfoData(QString infodataPath,bool matState = false);

	//[!]返回infodatatemp文件夹的路径
	static QString getInfoDataTempDataPath();
    //[!]返回fjslamtemp文件夹的路径
    static QString getFjSlamTempDataPath();
    //[!]清除infodatatemp文件夹
	static void cleanInfoDataFiles();
    //[!]清除fjslamtemp文件夹
    static void cleanFJslamDataFiles();
    ///<summary> 设置是否为红色警告样式 </summary>
    static void setLineEditStyle(QLineEdit * edit, bool isvalid);
	//[!]源路径，目标路径，拷贝文件名，是否加上基础
	static QMap<QString, QString> copyFilesAndReturnPaths(const QString& source_directory, const QString& destination_directory, const QStringList& file_names);
    //[!]转换编码
    static std::wstring to_wstring(const std::string &str_in);

    static bool file_exist(std::string file_name);

    static std::ifstream get_ifstream(std::string file_name);

    static std::ifstream get_ifstream_binary(std::string file_name);

    static QString Utf8ToQStr(const char* s);

	static std::string convertQStringPathToStringPath(QString path);
private:

	FJPointCloudUtil(void);
	~FJPointCloudUtil(void);
};

#endif // __FJPOINTCLOUDUTIL_H__
