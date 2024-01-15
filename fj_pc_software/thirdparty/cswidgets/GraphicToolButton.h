#pragma once
#include "cswidgets/icenterwidget.h"
#include <QToolbutton>

namespace CS {
    namespace Widgets {

		class CSWIDGETS_EXPORT GraphicToolButton : public QToolButton
		{
			Q_OBJECT
		public:
			explicit GraphicToolButton(QWidget *parent = nullptr);
		};
	}
}
