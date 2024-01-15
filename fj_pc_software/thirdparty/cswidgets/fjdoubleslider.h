#ifndef FJDOUBLESLIDER_H
#define FJDOUBLESLIDER_H

#include <QWidget>
#include <QMouseEvent>
#include <QPaintEvent>
#include "cswidgets_global.h"
class CSWIDGETS_EXPORT FJDoubleSlider : public QWidget
{
    Q_OBJECT

public:
    enum CurrentSpanHandle
    {
        NoHandle,
        LowerHandle,
        UpperHandle
    };
    FJDoubleSlider(QWidget *parent = nullptr);
    ~FJDoubleSlider();

	/**
	* @brief 获取双滑块条较小值
	*/
	double lowerValue();

	/**
	* @brief 获取双滑块条较大值
	*/
	double upperValue();

	/**
	* @brief 设置双滑块条范围
	*/
    void setQxtSpanRange(double lower, double upper){m_Min = lower;m_Max = upper;}

	/**
	* @brief 更新双滑块条位置
	*/
    void updatePos();

public slots:

	/**
	* @brief 设置双滑块条较小值
	*/
    void setLowerValue(double lower);

	/**
	* @brief 设置双滑块条较大值
	*/
    void setUpperValue(double upper);

	/**
	* @brief 设置双滑块条较小较大值
	*/
    void setSpan(double lower, double upper);

signals:
    void lowerValueChanged(double lower);
    void upperValueChanged(double upper);


protected:
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void paintEvent(QPaintEvent* event);
private:
    double m_Lower = 0.0;
    double m_Upper = 0.0;
    double m_LowerPos = 0.0;
    double m_UpperPos = 0.0;
    double m_Min = 0.0;
    double m_Max = 99.0;
    CurrentSpanHandle m_Handle = NoHandle;
};
#endif // FJDOUBLESLIDER_H
