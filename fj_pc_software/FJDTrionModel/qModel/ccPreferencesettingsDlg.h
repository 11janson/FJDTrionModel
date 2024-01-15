//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_PREFERENCESETTINGSDLG
#define CC_PREFERENCESETTINGSDLG

#include <QDialog>
#include <framelessdialog.h>
namespace Ui {
	class PreferencesettingsDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class ccPreferencesettingsDlg : public CS::Widgets::FramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPreferencesettingsDlg(QWidget* parent = nullptr);
	
	~ccPreferencesettingsDlg();
	void initSetting();


	int Getlength();
	void Setlength(int num);

	int Getdiameter();
	void Setdiameter(int num);

	int Getangle();
	void Setangle(int num);

	int Getarea();
	void Setarea(int num);

	int Getvolume();
	void Setvolume(int num);

	int Getdecimal();
	void Setdecimal(int num);

	bool GetShowunitlabels();
	void SetShowunitlabels(bool ischecked);

	int Gettransparency();
	void Settransparency(int num);

	int GetLabelfontsize();
	void SetLabelfontsize(int num);

	int GetMarkersize();
	void SetMarkersize(int num);

	QColor GetLabelColor();
	void SetLabelColor(QColor color);

	QColor GetMarkerColor();
	void SetMarkerColor(QColor color);

	int GetPrompttextsize();
	void SetPrompttextsize(int num);

	bool Getdrawroundpoints();
	void Setdrawroundpoints(bool isdrawround);

	bool GetDisplaycrossmark();
	void SetDisplaycrossmark(bool ischecked);

    int getSettingPointSize();
    bool setSettingPointSize(int);
public:
	/**
	*@brief 设置渲染亮度
	*/
	void setRenderBrightnessValue(int nValue);
	/**
	*@brief 获取渲染亮度值
	*/
	int getRenderBrightnessValue(void);
	/**
	*@brief 设置渲染对比度值
	*/
	void settRenderContrastValue(int nValue);
	/**
	*@brief 获取渲染对比度值
	*/
	int getRenderContrastValue(void);

	/**
	*@brief 如果发生变化，重置对比度亮度只
	*/
	void resetContrastBrightness(const int &iBrightness, const int &iContrast);

	/**
	*brief 更新RGB
	*/
	void updateRgbContrastBrightness(int iBrightness, int iContrast);

	
public slots:
	void shiftWindow(bool checked);
	void changeFirstColor(bool checked);
	void changeSecondColor(bool checked);
	void initParamSetting(bool checked);
	void addClassfication();
	/**
	*@brief 对比度亮度发生变化
	*/
	void slotRenderBrightnessContrastChanged(void);
private:
	Ui::PreferencesettingsDlg* m_ui;

	QColor m_LabelColor;
	QColor m_MarkerColor;
	QString m_selectedstyle = "";
	QString m_disselectedstyle = "";
	
};

#endif //CC_SOR_FILTER_DLG_HEADER
