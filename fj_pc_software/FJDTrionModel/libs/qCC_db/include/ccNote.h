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
    //! 设置相对位置
    void setPosition(float x, float y);

    /**
  *@brief 获取raw名称
  */
    inline QString getRawName() const { return m_name; }

    //! Clears label
    void clear(bool ignoreDependencies = false);

public://继承基类部分

    /**
    *@brief 获取节点名称
    */
    virtual QString getName() const override;

    /**
    *@brief 获取注册的数据类型
    */
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::NOTE_2D; }

    /**
    *@brief 是否需要串行化数据
    */
    inline virtual bool isSerializable() const override { return true; }

    /**
    *@brief 接收鼠标选择
    */
    virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;

    /**
    *@brief 鼠标移动位置计算
    */
    virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

    /**
    *@brief 绘画还是
    */
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
   

    virtual void onDeletionOf(const ccHObject* obj) override;


    virtual bool toFile_MeOnly(QFile& out) const override;
    virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

 

  
public:
    /**
   *@brief 绘画2d效果
   */
    void drawMeOnly2D(CC_DRAW_CONTEXT& context);
    /**
    *@brief 绘画3d效果
    */
    void drawMeOnly3D(CC_DRAW_CONTEXT& context);
    /**
    *@brief 设置选择模式
    */
    void setPickingMode(Mode pickingMode);

    /**
    *@brief 设置当前拾取的点
    */
   void setPickedPoint(const cc2DLabel::PickedPoint& pp);

   /**
   *@brief 获取拾取点的位置
   */
   cc2DLabel::PickedPoint &getPickedPoint(void);

   /**
   *@brief 设置标题
   */
   void setTitle(const QString strTitle);

   /**
   *@brief 获取标题
   */
   QString getTitle(void);

   /**
   *@brief 设置选择中
   */
   void setContent(const QString strContext);

   /**
   *@brief 获取内容
   */
   QString getContent(void);

   /**
   *@brief 获取注释图片
   */
   std::vector<QImage> getNoteImages(void);

   /**
   *@brief 设置注释图片
   */
   void setNoteImages(std::vector<QImage> listImage);

   /**
   *@brief 添加图片
   */
   bool appendImage(const QImage &image);

private:
    QString m_strTitle = QString("Note");                       ///<标题
    QString m_strContent = QString::null;                       ///<标题内容

private:
    cc2DLabel::PickedPoint  m_stPickedPoint;                    ///<当前拾取的点            
    Mode m_pickingMode;                                         ///<选择模式 
    float m_relMarkerScale    = 1.0f;
    //! Whether to display the label in 2D
    bool m_dispIn2D = true;
    bool m_dispPointsLegend  = false;
private:
    QImage m_activaImage;                                       ///<激活图片
    QImage m_normalImage;                                       ///<常态图片

private:
    QRect m_labelROI;
    int m_ButtonROI[4];
private:
    float  m_screenPos[2];                                      ///<位置
    int m_lastScreenPos[2];
 private:
     std::vector<QImage>    m_listNoteImages;
     QPoint m_offsetPoint = QPoint(0, 0);  

};