#pragma  once
#include <QColor>
#include <QPoint>
#include "ccHObject.h"
#include "ccInteractor.h"



class QCC_DB_LIB_API ccStaticLabel : public ccHObject, public ccInteractor
{
public:
    ccStaticLabel(QString strName = "StaticLabel");
    /**
   *@brief 获取raw名称
   */
    inline QString getRawName() const { return m_name; }

public:
    /**
    *@brief 设置位置
    */
    void setPosition(CCVector3d positon);

    /**
   *@brief 获取位置
   */
    CCVector3d getPositon(void);

    /**
    *@设置选择中的时候颜色
    */
    void setSelectColor(QColor color);

    /**
    *@设置常态下的颜色
    */
    void setNormalColor(QColor color);

    /**
    *@brief 设置标题内容
    */
    void setTitleContent(const QString &strTitleContent);

public://继承基类部分
/**
*@brief 获取节点名称
*/
    virtual QString getName() const override;

    /**
    *@brief 获取注册的数据类型
    */
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::STATUC_LABEL; }

    /**
    *@brief 接收鼠标选择
    */
    virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;

    /**
    *@brief 鼠标移动位置计算
    */
    virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

    //! Called on mouse move (for 3D interactors)
    /** \return true if a movement occurs
    **/
    virtual bool move3D(const CCVector3d& u)override;

    /**
    *@brief 绘画还是
    */
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
public:
    /**
   *@brief 绘画2d效果
   */
    void drawMeOnly2D(CC_DRAW_CONTEXT& context);
    /**
    *@brief 绘画3d效果
    */
    void drawMeOnly3D(CC_DRAW_CONTEXT& context);


private:
    float m_relMarkerScale = 1.0f;
    QPoint m_drawPoint = QPoint(0, 0);
    CCVector3d m_Position;                                         ///<绘制时候3D位置
    CCVector3d m_Position2D;           ///<2D位置 
    QColor m_selecColor = QColor(128, 128, 0);                    ///<选中中的时候颜色
    QColor m_normlColor = QColor(255, 0, 0);                      ///< 常态下的颜色
private:
    QPointF m_TranslatefPoint = QPointF(0, 0);
    float  m_screenPos[2];                                      ///<位置
private:
    QString m_titleContent = QString::null;
    QString m_strName = QString::null;
};