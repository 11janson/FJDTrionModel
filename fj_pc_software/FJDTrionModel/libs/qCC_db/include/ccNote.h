#pragma once
//Local
#include "ccHObject.h"
#include "ccInteractor.h"
#include "ccGenericGLDisplay.h"
#include "cc2DLabel.h"
//Qt
#include <QRect>
#include <QObject>
#include <QImage>
#include <vector>
#include <QVector>

class ccGenericPointCloud;
class ccGenericMesh;

class QCC_DB_LIB_API ccNote : public ccHObject, public ccInteractor
{
public:
    ccNote(QString strName = QString("Note"));


public:
    //! �������λ��
    void setPosition(float x, float y);

    /**
  *@brief ��ȡraw����
  */
    inline QString getRawName() const { return m_name; }

    //! Clears label
    void clear(bool ignoreDependencies = false);

public://�̳л��ಿ��

    /**
    *@brief ��ȡ�ڵ�����
    */
    virtual QString getName() const override;

    /**
    *@brief ��ȡע�����������
    */
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::NOTE_2D; }

    /**
    *@brief �Ƿ���Ҫ���л�����
    */
    inline virtual bool isSerializable() const override { return true; }

    /**
    *@brief �������ѡ��
    */
    virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;

    /**
    *@brief ����ƶ�λ�ü���
    */
    virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

    /**
    *@brief �滭����
    */
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
   

    virtual void onDeletionOf(const ccHObject* obj) override;


    virtual bool toFile_MeOnly(QFile& out) const override;
    virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

 

  
public:
    /**
   *@brief �滭2dЧ��
   */
    void drawMeOnly2D(CC_DRAW_CONTEXT& context);
    /**
    *@brief �滭3dЧ��
    */
    void drawMeOnly3D(CC_DRAW_CONTEXT& context);
    /**
    *@brief ����ѡ��ģʽ
    */
    void setPickingMode(Mode pickingMode);

    /**
    *@brief ���õ�ǰʰȡ�ĵ�
    */
   void setPickedPoint(const cc2DLabel::PickedPoint& pp);

   /**
   *@brief ��ȡʰȡ���λ��
   */
   cc2DLabel::PickedPoint &getPickedPoint(void);

   /**
   *@brief ���ñ���
   */
   void setTitle(const QString strTitle);

   /**
   *@brief ��ȡ����
   */
   QString getTitle(void);

   /**
   *@brief ����ѡ����
   */
   void setContent(const QString strContext);

   /**
   *@brief ��ȡ����
   */
   QString getContent(void);

   /**
   *@brief ��ȡע��ͼƬ
   */
   std::vector<QImage> getNoteImages(void);

   /**
   *@brief ����ע��ͼƬ
   */
   void setNoteImages(std::vector<QImage> listImage);

   /**
   *@brief ���ͼƬ
   */
   bool appendImage(const QImage &image);

private:
    QString m_strTitle = QString("Note");                       ///<����
    QString m_strContent = QString::null;                       ///<��������

private:
    cc2DLabel::PickedPoint  m_stPickedPoint;                    ///<��ǰʰȡ�ĵ�            
    Mode m_pickingMode;                                         ///<ѡ��ģʽ 
    float m_relMarkerScale    = 1.0f;
    //! Whether to display the label in 2D
    bool m_dispIn2D = true;
    bool m_dispPointsLegend  = false;
private:
    QImage m_activaImage;                                       ///<����ͼƬ
    QImage m_normalImage;                                       ///<��̬ͼƬ

private:
    QRect m_labelROI;
    int m_ButtonROI[4];
private:
    float  m_screenPos[2];                                      ///<λ��
    int m_lastScreenPos[2];
 private:
     std::vector<QImage>    m_listNoteImages;
     QPoint m_offsetPoint = QPoint(0, 0);  

};