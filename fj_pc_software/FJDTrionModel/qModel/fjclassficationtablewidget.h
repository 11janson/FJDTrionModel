#ifndef FJCLASSFICATIONTABLEWIDGET_H
#define FJCLASSFICATIONTABLEWIDGET_H

#include <QWidget>
#include <QColor>
#include "fjpointcloudutil.h"
namespace Ui {
class FJClassficationTableWidget;
}

class FJClassficationTableWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FJClassficationTableWidget(QWidget *parent = nullptr);
    ~FJClassficationTableWidget();

    void initClassficationColorSetting(std::vector<ClassficationData> data);

	std::vector<ClassficationData> getClassficationColor();

signals:
	void dataChanged();
private:
    Ui::FJClassficationTableWidget *ui;
	std::vector<ClassficationData> m_colorData;
};

#endif // FJCLASSFICATIONTABLEWIDGET_H
