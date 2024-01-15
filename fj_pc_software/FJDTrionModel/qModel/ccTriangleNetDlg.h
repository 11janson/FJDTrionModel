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

#ifndef CC_CC_TRIANGLE_DLG_HEADER
#define CC_CC_TRIANGLE_DLG_HEADER


#include <ui_triangleDlg.h>
#include <framelessdialog.h>

struct surfaceMeshPara {
    int methodType;//表面三角网构建算法
    double subFactor;//原始点云采样间隔因子
};

//! Dialog to define connected components labelinng parameters
class ccTriangleDlg : public CS::Widgets::FramelessDialog, public Ui::TriangleDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccTriangleDlg(QWidget* parent = nullptr);


    surfaceMeshPara getSurfaceMeshPara();

};

#endif //CC_CC_TRIANGLE_DLG_HEADER
