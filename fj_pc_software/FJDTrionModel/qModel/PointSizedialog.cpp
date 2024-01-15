#include "PointSizedialog.h"
#include <QVBoxLayout>
#include <QComboBox>
#include <QLabel>
#include <QCoreApplication>
#include "ccGuiParameters.h"
#include <QListView>
PointSizedialog::PointSizedialog(QWidget *parent, Qt::WindowFlags f)
    : FJBaseWidget(parent, f | Qt::X11BypassWindowManagerHint)
{
	setContextMenuPolicy(Qt::NoContextMenu);
	QVBoxLayout* psLayout = new QVBoxLayout;
	psLayout->setSpacing(0);

    QStringList pointSizeStr = (QStringList() << QCoreApplication::translate("MainWindow", "Auto", nullptr) << "1"
        << "2" << "3" << "4" << "5" << "6" << "7" << "8" << "9" << "10" << "11" << "12" << "13" << "14" << "15" << "16");
    pointSize_Box = new QComboBox(this);
    //pointSize_Box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pointSize_Box->addItems(pointSizeStr);
    pointSize_Box->setObjectName("actionpointsizecombox");

    m_pPointSizeLabel = new QLabel(this);
    m_pPointSizeLabel->setText(QCoreApplication::translate("MainWindow", "Point size", nullptr));
    m_pPointSizeLabel->setStyleSheet("");
    m_pPointSizeLabel->setStyleSheet(QString("QLabel{border: none; background - color: transparent;}QLabel:disabled{color: #838383;}"));
    m_pPointSizeLabel->setToolTip(QCoreApplication::translate("MainWindow", "Set point size", nullptr));
    m_pPointSizeLabel->setObjectName("actionpointsizelable");
    m_pPointSizeLabel->setFixedHeight(20);
    pointSize_Box->setCurrentIndex(0);

	connect(pointSize_Box, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), [=](int index) {
		emit currentindexchange(index);
	});
	psLayout->addWidget(pointSize_Box);
	psLayout->addStretch(2);
	psLayout->addWidget(m_pPointSizeLabel);
	psLayout->addStretch(5);
	psLayout->setContentsMargins(10, 12, 0, 0);
	setLayout(psLayout);
	setObjectName("actionpointsize");

}

PointSizedialog::~PointSizedialog()
{

}

int PointSizedialog::getComboBoxIndex()
{
    return pointSize_Box->currentIndex();
}

QComboBox* PointSizedialog::getComBoBox()
{
    return pointSize_Box;
}

void PointSizedialog::setUiEnabled(bool state)
{
    m_pPointSizeLabel->setEnabled(state);
    pointSize_Box->setEnabled(state);
    if (!state) {
        pointSize_Box->setCurrentIndex(ccGui::Parameters().m_globalpointsize);
    }
}

void PointSizedialog::InitFJStyle()
{
}

void PointSizedialog::paintEvent(QPaintEvent *event)
{

}

