#pragma once
#include "cswidgets_global.h"
#include <QFrame>
#include <QUuid>
#include <memory>
#include "csutils/id.h"
namespace CS {
namespace Widgets {
class CSWIDGETS_EXPORT ICenterWidget : public QFrame
{
    Q_OBJECT
public:
    explicit ICenterWidget(Utils::Id id,QWidget *parent = 0);
    virtual ~ICenterWidget();

	/***
	*@brief 返回当前窗口ID
	*/
	Utils::Id id() const { return m_id; }

    /**
	*@brief 激活窗口
	*/
    void raiseWindow();
	/**
	*@brief 设置窗口Modelid
	*/
    virtual void setMode(Utils::Id);
	/**
	*@brief 进入窗口
	*/
    virtual void enterWindow();
	/**
	*@brief 离开窗口
	*/
    virtual void leaveWindow();
    /**
    * @brief 实时翻译时，刷新UI文字显示
    */
    virtual void retranslateUi();

signals:
    /**
    *@brief 窗口关闭
    */
    void signalCloseWindow(void);
protected:
    virtual void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;
    virtual void showEvent(QShowEvent *event) Q_DECL_OVERRIDE;
    virtual void changeEvent(QEvent *e) Q_DECL_OVERRIDE;
private:
    void createWidgets();
    void createConnects();
protected:
private:
    Utils::Id       m_id;
};
}
}
