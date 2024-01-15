#pragma  once
#include <QRect>
#include <QImage>
#include "ccHObject.h"
#include "ccInteractor.h"


class QCC_DB_LIB_API ccPoint : public ccHObject, public ccInteractor
{
public:
    ccPoint(QString strName = QString("Point"));
    /**
    *@brief 获取raw名称
    */
    inline QString getRawName() const { return m_name; }

public:
    /**
    *@brief 设置位置
    */
    void setPosition(CCVector3d positon);


    void setScreenPos(QPointF pos);

    /**
    *@brief 获取位置
    */
    CCVector3d getPositon(void);

    CCVector3d get2DPostion(void);

public://继承基类部分
    /**
    *@brief 获取节点名称
    */
    virtual QString getName() const override;

    /**
    *@brief 获取注册的数据类型
    */
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POINT_C; }

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
    
    QPointF getTranslatefPoint(void) { return m_TranslatefPoint; };
    QPoint getDrawPoint(void) { return m_drawPoint; };
    void setShow(bool show) { m_show = show; };
 private:
    QPointF m_TranslatefPoint = QPointF(0, 0);
    QPoint m_drawPoint = QPoint(0, 0);

    float m_relMarkerScale = 1.0f;
    CCVector3d m_Position;             ///<位置
    CCVector3d m_Position2D;           ///<2D位置       
    QRect   m_stRect;                ///<区域
    QString m_strName = QString::null;
private:
    float  m_screenPos[2];                                      ///<位置
    int m_lastScreenPos[2];

    bool m_show;
private:
    QImage m_activaImage;                                       ///<激活图片
    QImage m_normalImage;                                       ///<常态图片

};