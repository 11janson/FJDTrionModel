#pragma once
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

#include "qCC_glWindow.h"

//Qt
#include <QString>

//qCC_db
#include <ccColorTypes.h>

/***************************************************
				GUI parameters
***************************************************/

//! This class manages some persistent parameters (mostly for display)
/** Values of persistent parameters are stored by the system
	(either in the registry or in a separate file depending on the OS).
**/
class CCGLWINDOW_LIB_API ccGui
{
public:

	//! GUI parameters
	struct CCGLWINDOW_LIB_API ParamStruct
	{
		//! Light diffuse color (RGBA)
		ccColor::Rgbaf lightDiffuseColor;
		//! Light ambient color (RGBA)
		ccColor::Rgbaf lightAmbientColor;
		//! Light specular color (RGBA)
		ccColor::Rgbaf lightSpecularColor;
		//! Double sided light
		bool lightDoubleSided;

		//! Default mesh diffuse color (front)
		ccColor::Rgbaf meshFrontDiff;
		//! Default mesh diffuse color (back)
		ccColor::Rgbaf meshBackDiff;
		//! Default mesh specular color
		ccColor::Rgbaf meshSpecular;

		//! Default text color
		ccColor::Rgba textDefaultCol;
		//! Default 3D points color
		ccColor::Rgba pointsDefaultCol;
		//! Background color
		ccColor::Rgbub backgroundCol;
		//! Labels background color
		ccColor::Rgba labelBackgroundCol;
		//! Labels marker color
		ccColor::Rgba labelMarkerCol;
		//! Bounding-boxes color
		ccColor::Rgba bbDefaultCol;
		
		//! Use background gradient
		bool drawBackgroundGradient;
		//! Decimate meshes when moved
		bool decimateMeshOnMove = true;
		//! Min mesh size for decimation
		unsigned minLoDMeshSize = 2500000;;
		//! Decimate clouds when moved
		bool decimateCloudOnMove;
		//! Min cloud size for decimation
		unsigned minLoDCloudSize;
		//! Display cross in the middle of the screen
		bool displayCross;
		//! Whether to use VBOs for faster display
		bool useVBOs;

		//! Label marker size
		unsigned labelMarkerSize;

		//! Color scale option: show histogram next to color ramp
		bool colorScaleShowHistogram;
		//! Whether to use shader for color scale display (if available) or not
		bool colorScaleUseShader;
		//! Whether shader for color scale display is available or not
		bool colorScaleShaderSupported;
		//! Color scale ramp width (for display)
		unsigned colorScaleRampWidth;

		//! Default displayed font size
		unsigned defaultFontSize;
		//! Label font size
		unsigned labelFontSize;
		//! Displayed numbers precision
		unsigned displayedNumPrecision;
		//! Labels background opcaity
		unsigned labelOpacity;

		//! Zoom speed (1.0 by default)
		double zoomSpeed;

		//! Octree computation (for picking) behaviors
		enum ComputeOctreeForPicking { ALWAYS = 0, ASK_USER = 1, NEVER = 2 };

		//! Octree computation (for picking) behavior
		ComputeOctreeForPicking autoComputeOctree;

		//���ȵ�λö��
		enum Lengthparametertype { RICE = 0, DECIMETER = 1, CENTIMETER = 2, MILLIMETER = 3, KILOMETRE = 4, MILE = 5, FOOT = 6, INCH = 7};

		//�Ƕȵ�λö��
		enum Angleparametertype { DEGREE = 0, BRANCH = 1, SECOND = 2};

		//�����λö��
		enum Areaparametertype { SQUAREMETER = 0, SQUAREDECIMETER = 1, SQUARECENTIMETRE = 2, SQUAREMILLIMETRE = 3, SQUAREFOOT = 4, SQUAREINCH = 5};

		//�����λö��
		enum Volumnparametertype { CUBICMETER = 0, CUBICDECIMETER = 1, CUBICCENTIMETRE = 2, CUBICMILLIMETRE = 3, CUBICFOOT = 4, CUBICINCH = 5};


		Lengthparametertype lengthunit;
		Lengthparametertype diameterunit;
		Angleparametertype angleunit;
		Areaparametertype areaunit;
		Volumnparametertype volumnunit;
		bool isshowUnitlabel;

		//! Whether to draw rounded points (slower) or not
		bool drawRoundedPoints;

		//! Whether to pick objects by single clicking in the GUI 
		//! can be slow with large point clouds/large number of objects
		bool singleClickPicking;

        bool m_bNotePickGLEnable = false;
        bool m_bShowCapacityPoint = false; // ��ʾ������
        bool m_bGraphicsElement = false;   //��άʸ����ͼ
		bool m_bCustomMouseStyle = false;	///< 自定义鼠标样式
        bool m_bViewRotateEanble = true;
		bool m_openShiftMeasure = true; //<�Ƿ�shift�����������
        bool m_bBubbleRotationModeOn = true;
		bool m_isSectionAnalysis = false;//<������������Ƿ���
		bool m_isManualClassificationOpen = false; //<�ֶ����๦���Ƿ���
		bool m_isTwoPointCuttingOn = false; //����ָ���Ƿ���
		bool m_isMeasureOn = false; //���������Ƿ���
		bool m_isMousePointerStyle = false;//[!]����Ϊtrue������������GLwindowʱ����Ϊʰȡ��ʽ���뿪��Ϊָ����ʽ��
        bool m_isOrthDraw = false;
        bool m_isHoverSelected = false;//ͼԪ���ư�ť����״̬
		float m_globalpointsize = 1;

        bool m_enableCrossSelect = false; 
        bool m_isPlaneCut = false;
        bool m_ignoreSelectPoint = false;
		bool m_isPaintPageActive = false;
		bool m_isPickGLEnable = true;
		bool m_bGraphicsPickStatus = false;
		
        bool m_currentHoverSelectItem = false;
        bool m_is2DView = false;
		int m_nBrightness = 10;
		int m_nContrast = 25;
        //[!]判断裁切盒功能是否开启
        bool m_CuttingBoxisOpen = false;
        //[!]当鼠标按下的时候就要关闭裁切盒的拾取，当鼠标恢复的时候打开，相当于次级的控制
        bool m_CuttingBoxPickingOpen = true;
		
        bool m_canChangePivotPoint = true;  //是否可以移动旋转中心位置
        bool m_isImageMeshOnlyShow = false; //是否只显示图像融合的图像
		ParamStruct();                      //! Default constructor

		//! Resets parameters to default values
		void reset();

		//! Loads from persistent DB
		void fromPersistentSettings();

		//! Saves to persistent DB
		void toPersistentSettings() const;

		//! Returns whether a given parameter is already defined in persistent settings or not
		/** \param paramName the corresponding attribute name
		**/
		bool isInPersistentSettings(QString paramName) const;
	};

	//! Returns the stored values of each parameter.
	static const ParamStruct& Parameters();

	//! Sets GUI parameters
	static void Set(const ParamStruct& params);

	//! Release unique instance (if any)
	static void ReleaseInstance();

protected:

	//! Parameters set
	ParamStruct params;

};
