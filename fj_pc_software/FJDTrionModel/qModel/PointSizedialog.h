#ifndef POINTSIZEDIALOG_H
#define POINTSIZEDIALOG_H

#include <FJBaseWidget.h>
#include <QDialog>
#include <QPaintEvent>
#include <QComboBox>
#include <QLabel>
class PointSizedialog : public FJBaseWidget
{
	Q_OBJECT
public:
	explicit PointSizedialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
	virtual ~PointSizedialog();

	virtual void InitFJStyle();
    int getComboBoxIndex();
    QComboBox* getComBoBox();
	void setUiEnabled(bool state);
signals:
	void currentindexchange(int index);
protected:
	virtual void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
	QComboBox* pointSize_Box;
	QLabel* m_pPointSizeLabel = nullptr;
};

#endif 