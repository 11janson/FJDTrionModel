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
   *@brief ��ȡraw����
   */
    inline QString getRawName() const { return m_name; }

public:
    /**
    *@brief ����λ��
    */
    void setPosition(CCVector3d positon);

    /**
   *@brief ��ȡλ��
   */
    CCVector3d getPositon(void);

    /**
    *@����ѡ���е�ʱ����ɫ
    */
    void setSelectColor(QColor color);

    /**
    *@���ó�̬�µ���ɫ
    */
    void setNormalColor(QColor color);

    /**
    *@brief ���ñ�������
    */
    void setTitleContent(const QString &strTitleContent);

public://�̳л��ಿ��
/**
*@brief ��ȡ�ڵ�����
*/
    virtual QString getName() const override;

    /**
    *@brief ��ȡע�����������
    */
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::STATUC_LABEL; }

    /**
    *@brief �������ѡ��
    */
    virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;

    /**
    *@brief ����ƶ�λ�ü���
    */
    virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

    //! Called on mouse move (for 3D interactors)
    /** \return true if a movement occurs
    **/
    virtual bool move3D(const CCVector3d& u)override;

    /**
    *@brief �滭����
    */
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
public:
    /**
   *@brief �滭2dЧ��
   */
    void drawMeOnly2D(CC_DRAW_CONTEXT& context);
    /**
    *@brief �滭3dЧ��
    */
    void drawMeOnly3D(CC_DRAW_CONTEXT& context);


private:
    float m_relMarkerScale = 1.0f;
    QPoint m_drawPoint = QPoint(0, 0);
    CCVector3d m_Position;                                         ///<����ʱ��3Dλ��
    CCVector3d m_Position2D;           ///<2Dλ�� 
    QColor m_selecColor = QColor(128, 128, 0);                    ///<ѡ���е�ʱ����ɫ
    QColor m_normlColor = QColor(255, 0, 0);                      ///< ��̬�µ���ɫ
private:
    QPointF m_TranslatefPoint = QPointF(0, 0);
    float  m_screenPos[2];                                      ///<λ��
private:
    QString m_titleContent = QString::null;
    QString m_strName = QString::null;
};