#ifndef __FJCLASSFICATIONTABLECONFIGWIDGET_H__
#define __FJCLASSFICATIONTABLECONFIGWIDGET_H__

#include <QWidget>
#include <QColor>
#include "fjpointcloudutil.h"
namespace Ui {
class FJClassficationTableConfigWidget;
}

class FJClassficationTableConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FJClassficationTableConfigWidget(QWidget *parent = nullptr);
    ~FJClassficationTableConfigWidget();

	//加载类别颜色配置
    void initClassficationColorSetting(bool isdefault);

	//添加一行类别数据
	void addRowData();

	//初始化为默认配置
	void resetClassfication();

private:
	//添加一行类别数据
	void addRowParme(ClassficationData curdata);

	//应用配置到所有点云
	void applyAllPointCloud();
private:
    Ui::FJClassficationTableConfigWidget *ui;

};

#endif // FJCLASSFICATIONTABLEWIDGET_H
