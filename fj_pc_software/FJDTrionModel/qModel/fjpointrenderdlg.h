#ifndef FJPOINTRENDERDLG_H
#define FJPOINTRENDERDLG_H

#include <QWidget>
#include <FJBaseWidget.h>
#include <QToolButton>
#include "ccHObject.h"
QT_BEGIN_NAMESPACE
namespace Ui { class FJPointRenderDlg; }
QT_END_NAMESPACE

class FJPointRenderDlg : public FJBaseWidget
{
    Q_OBJECT

public:

    FJPointRenderDlg(QWidget *parent = nullptr);
    ~FJPointRenderDlg();

    void setCurrentSelectObject(ccHObject::Container selectedEntities, bool buttonClick = true);

    virtual void InitFJStyle();

    void showColorByScalar(QString scalarName);
	
    void applySelectedMode(const CurrentCombinationMode & currentMode, const std::vector<CurrentCombinationMode> & supportModeVec);

    //aric.tang_2022.9.29
    void setColorScale(bool isuniquecolor);

    std::vector<CurrentCombinationMode> getSupportModes(ccHObject *entity);

    void setActionIcon(QToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix);

    void setToolButtonStyle(QToolButton * btn, bool isSelected);

	void setAllButtonDisabled();

signals:
    void refreshAll(bool only2D = false);

    void updateRibbonStripAction();
private:
    Ui::FJPointRenderDlg *ui;

    ccHObject::Container m_selectedEntities;
    bool m_isShowEDL = false;
};
#endif // FJPOINTRENDERDLG_H
