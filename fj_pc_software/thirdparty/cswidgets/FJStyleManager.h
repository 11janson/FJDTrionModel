#ifndef FJSTYLEMANAGER_H
#define FJSTYLEMANAGER_H
#include "cswidgets_global.h"
#include <QObject>
#include <QString>
#include "FJBaseWidget.h"

/**
*@brief 不同皮肤颜色主题枚举
*/
enum ThemeStyleType
{
    Black = 0,
    White = 1,
    Blue = 2
};
class CSWIDGETS_EXPORT FJStyleManager : public QObject
{
    Q_OBJECT
public:
    static FJStyleManager* Instance(){
        return instance;
    }

    static void DestoryInstance()
    {
        if (instance != NULL )
        {
            delete instance;
            instance = NULL ;
        }
    }

	/**
	*@brief 获取当前肤色主题资源文件夹，获取图标时可以用此函数获取文件夹名，不同肤色对应图标名字保持一致
	*/
    QString Getcurrentthemepath();

	/**
	*@brief 切换肤色主题接口，可以配合切换qss文件一起使用
	*/
    void ShiftStyle(ThemeStyleType type);

	/**
	*@brief 获取当前肤色主题
	*/
    ThemeStyleType GetcurrentStyle();

	/**
	*@brief 管理继承FJBaseWidget类样式，目前通过信号槽连接
	*/
    void AddManagerWidget(FJBaseWidget * widget);

	/**
	*@brief 断开所有信号槽连接
	*/
    void DestoryConnect();

	/**
	*@brief 更新平行投影或者透视投影
	*/
	void updatePerspectiveState();

	/**
	*@brief 绘制类选中
	*/
	void areaMeasureLabelSelected(int uuid);

	/**
	*@brief 剖面厚度更新
	*/
	void updateSectionThickness(QString name, double thick);

    /**
    *@brief 删除顶点数组对象
    */
    void deleteVertexArrays(unsigned int id);

    /**
    *@brief 删除缓冲对象
    */
    void deleteBuffers(unsigned int id);

    /**
    *@brief 删除点云对象
    */
    void removePointCloudObject(int uuid);
	
    /**
    *@brief 界面报错
    */
    void showErrorMessage(QString mes);

    void updateMappingPointSize(bool state,int size);

signals:
    void ThemeStyleShift();

	void measureLabelSelected(int uuid);

	void updatePerspectiveStateSignal();

	void SectionThicknessUpdate(QString name,double thick);

    void updateCloudPointSizeSignal();

    void signalDeleteVertexArrays(unsigned int id);

    void signalDeleteBuffers(unsigned int id);

    void pointcloudRemoved(int uuid);

    void signalErrorMessage(QString mes);
    /**
    *@brief ˢ�µ��ƵĴ�С
    */
    void signalUpdateMappingPointSize(bool,int);

private:
    FJStyleManager(){};
    ~FJStyleManager(){};
    static FJStyleManager* instance ;
    ThemeStyleType m_currenttype = Black;

};


#endif // FJSTYLEMANAGER_H
