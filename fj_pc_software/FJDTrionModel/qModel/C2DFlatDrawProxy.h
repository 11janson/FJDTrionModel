#pragma  once

#include <QObject>

#include <cc2DItemBase.h>

class ccPointCloud;
class ccLineGroup;
class ccGLWindow;
class ccPolyline;
class ccLines;
class ccHObject;

class C2DFlatDrawProxy : public QObject
{
	Q_OBJECT

public:
public:
	C2DFlatDrawProxy();
	~C2DFlatDrawProxy();

	bool linkWith(ccGLWindow* win);

	void setGraphicType(cc2DItemBase::EGraphicType type);
protected:
	//! Process states
	enum ProcessStates
	{
		POLYLINE = 1,
		RECTANGLE = 2,
		//...			= 4,
		//...			= 8,
		//...			= 16,
		PAUSED = 32,
		STARTED = 64,
		RUNNING = 128,
	};

	enum CurrentSegmentMode {
		RECTANGULAR = 0,
		POLYLINEMODE
	};


protected:
	void addPointToPolyline(int x, int y) { return addPointToPolylineExt(x, y, false); }
	void addPointToPolylineExt(int x, int y, bool allowClicksOutside);
	void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void closeRectangle();
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);


private:
	//! Segmentation polyline
	cc2DItemBase* m_pLineGroup;
	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

	ccGLWindow* m_associatedWin;

	ccHObject* m_pContainer;//存储多个二维图形

	//! Current process state
	unsigned m_state;

	//! Selection mode
	bool m_rectangularSelection;

	cc2DItemBase::EGraphicType m_Type;

	CurrentSegmentMode m_segmentMode = POLYLINEMODE;

	bool m_releaseMouse;
};

