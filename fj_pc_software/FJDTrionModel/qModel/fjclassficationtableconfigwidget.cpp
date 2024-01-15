#include "ui_fjclassficationtableconfigwidget.h"
#include <QCheckBox>
#include <QToolButton>
#include <QColorDialog>
#include "framelessdialog.h"
#include "mainwindow.h"
#include "fjclassficationtableconfigwidget.h"
#include "fjpointcloudutil.h"
#include "ccDBRoot.h"
#include "ccGLWindow.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"
FJClassficationTableConfigWidget::FJClassficationTableConfigWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FJClassficationTableConfigWidget)
{
    ui->setupUi(this);
	ui->tableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
	ui->tableWidget->verticalHeader()->setVisible(false);
	ui->tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	connect(ui->tableWidget, &QTableWidget::itemChanged, [=](QTableWidgetItem *item) {
		if (item->column() == 1)
		{
			int id = (ui->tableWidget->item(item->row(),0)->text()).toInt();
			FJPointCloudUtil::setNameByClassficationID(id, item->text());
		}
	});
	ui->tableWidget->setColumnWidth(0, 98);
	ui->tableWidget->setColumnWidth(1, 323);
	ui->tableWidget->setColumnWidth(2, 105);
}

FJClassficationTableConfigWidget::~FJClassficationTableConfigWidget()
{
    delete ui;
}

static void SetButtonColor( QAbstractButton* button, const QColor &col )
{
    if ( button != nullptr )
    {
		button->setFixedSize(24, 24);
        button->setStyleSheet( QStringLiteral( "* { background-color: rgb(%1,%2,%3);border: 1px solid #989898; }" )
                               .arg( col.red() )
                               .arg( col.green() )
                               .arg( col.blue() )
                               );
    }
}

void FJClassficationTableConfigWidget::initClassficationColorSetting(bool isdefault)
{
	ui->tableWidget->blockSignals(true);
	while (ui->tableWidget->rowCount() != 0)
	{
		ui->tableWidget->removeRow(ui->tableWidget->rowCount() - 1);
	}
	ui->tableWidget->blockSignals(false);
	std::vector<ClassficationData> data = (isdefault ? FJPointCloudUtil::getInitClassficationData() : FJPointCloudUtil::getClassficationDataFromDoc());
    for(auto curdata : data)
    {
		addRowParme(curdata);
    }
}

void FJClassficationTableConfigWidget::addRowParme(ClassficationData curdata)
{
	ui->tableWidget->blockSignals(true);
	std::set<int> defaultData{ 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14 };
	int rowCount = ui->tableWidget->rowCount();
	ui->tableWidget->setRowCount(rowCount + 1);
	ui->tableWidget->setRowHeight(rowCount, 32);

	QTableWidgetItem* item1 = new QTableWidgetItem();
	item1->setFlags(item1->flags() & ~Qt::ItemIsEditable);
	item1->setData(Qt::EditRole, QString::number(curdata.id));
	item1->setTextAlignment(Qt::AlignCenter);
	ui->tableWidget->setItem(rowCount, 0, item1);

	QTableWidgetItem* item2 = new QTableWidgetItem();
	item2->setData(Qt::EditRole, curdata.name);
	ui->tableWidget->setItem(rowCount, 1, item2);
	if (defaultData.find(curdata.id) != defaultData.end())
	{
		item2->setFlags(item2->flags() & ~Qt::ItemIsEditable);
	}
	item2->setTextAlignment(Qt::AlignCenter);


	QToolButton* btn = new QToolButton();
	SetButtonColor(btn, curdata.color);
	QWidget *widget = new QWidget();
	QHBoxLayout *layout = new QHBoxLayout();
	layout->setMargin(0);//一定要有
	layout->addWidget(btn);
	layout->setAlignment(btn, Qt::AlignCenter);//控件在布局中居中显示
	widget->setLayout(layout);
	connect(btn, &QToolButton::clicked, [=]() {

		CS::Widgets::FramelessDialog outdlg(MainWindow::TheInstance());
        outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
            "font-size: 16px;\n"
            "padding-left: 5px;\n"
            "color: #F2F2F2;\n"
            "background-color:transparent;\n"
            "line-height: 24px;\n");
		outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
		QColorDialog colordlg(FJPointCloudUtil::getColorByClassficationID(curdata.id), this);
		connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
		colordlg.setOptions(QColorDialog::NoButtons);
		outdlg.SetContentHolder(&colordlg);
		if (!outdlg.exec())
		{
			return;
		}
		QColor newCol = colordlg.currentColor();
		if (newCol.isValid())
		{
			FJPointCloudUtil::setColorByClassficationID(curdata.id, newCol);
			SetButtonColor(btn, newCol);
			applyAllPointCloud();
		}
	});
	ui->tableWidget->setCellWidget(rowCount, 2, widget);
	ui->tableWidget->blockSignals(false);
}

void FJClassficationTableConfigWidget::addRowData()
{
	std::vector<ClassficationData> data = FJPointCloudUtil::getClassficationDataFromDoc();
	std::set<int> idset;
	for (const auto & curdata : data)
	{
		idset.insert(curdata.id);
	}
	int newid = data.size();
	for (int i = 0;i <= data.size();i++)
	{
		if (idset.find(i) == idset.end())
		{
			newid = i;
			break;
		}
	}
	ClassficationData newdata(newid, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(newid), true, QColor(std::rand() % 255, std::rand() % 255, std::rand() % 255));
	data.push_back(newdata);
	sort(data.begin(), data.end(), [&](ClassficationData & x, ClassficationData & y)->bool {return (x.id < y.id); });
	FJPointCloudUtil::saveClassficationDataToDoc(data);
	initClassficationColorSetting(false);
	applyAllPointCloud();
	int rowcount = ui->tableWidget->rowCount();
	for (int i = 0; i < rowcount;i++)
	{
		if (ui->tableWidget->item(i,0)->text().toInt() == newid)
		{
			ui->tableWidget->setCurrentCell(i,0);
			break;
		}
	}
}

void FJClassficationTableConfigWidget::resetClassfication()
{
	std::vector<ClassficationData> docdata = FJPointCloudUtil::getClassficationDataFromDoc();
	std::vector<ClassficationData> data = FJPointCloudUtil::getInitClassficationData();
	for (const auto & curdata : docdata)
	{
		if (curdata.id > 49)
		{
			data.push_back(curdata);
		}
	}
	FJPointCloudUtil::saveClassficationDataToDoc(data);
	initClassficationColorSetting(false);
	applyAllPointCloud();
}

void FJClassficationTableConfigWidget::applyAllPointCloud()
{
	FJPointCloudUtil::updateAllPointCloudClassficationDataFromDoc();
	if (MainWindow::TheInstance()->getActiveGLWindow())
	{
		MainWindow::TheInstance()->getActiveGLWindow()->redraw(false);
	}
}
