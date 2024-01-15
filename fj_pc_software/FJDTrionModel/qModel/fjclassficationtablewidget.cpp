#include "fjclassficationtablewidget.h"
#include "ui_fjclassficationtablewidget.h"
#include <QCheckBox>
#include <QToolButton>
#include <QColorDialog>
#include "framelessdialog.h"
#include "mainwindow.h"
FJClassficationTableWidget::FJClassficationTableWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FJClassficationTableWidget)
{
    ui->setupUi(this);
	ui->tableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
	ui->tableWidget->verticalHeader()->setVisible(false);
	ui->tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	connect(ui->tableWidget, &QTableWidget::itemChanged, [=](QTableWidgetItem *item) {
		if (item->column() == 2)
		{
			m_colorData[item->row()].name = item->text();
			emit dataChanged();
		}
	});
	ui->tableWidget->setColumnWidth(0, 80);
	ui->tableWidget->setColumnWidth(1, 100);
	ui->tableWidget->setColumnWidth(2, 125);
	ui->tableWidget->setColumnWidth(3, 163);
}

FJClassficationTableWidget::~FJClassficationTableWidget()
{
    delete ui;
}

static void SetButtonColor( QAbstractButton* button, const QColor &col )
{
    if ( button != nullptr )
    {
        button->setStyleSheet( QStringLiteral( "* { background-color: rgb(%1,%2,%3) }" )
                               .arg( col.red() )
                               .arg( col.green() )
                               .arg( col.blue() )
                               );
    }
}

void FJClassficationTableWidget::initClassficationColorSetting(std::vector<ClassficationData> data)
{
	std::set<int> defaultData{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
	while (ui->tableWidget->rowCount() != 0)
	{
		ui->tableWidget->removeRow(ui->tableWidget->rowCount() - 1);
	}
	m_colorData.clear();
	m_colorData = data;
	ui->tableWidget->blockSignals(true);
    for(auto curdata : data)
    {
        int rowCount = ui->tableWidget->rowCount();
        ui->tableWidget->setRowCount(rowCount + 1);
		ui->tableWidget->setRowHeight(rowCount, 32);
		QCheckBox* box = new QCheckBox();

		QWidget *widget = new QWidget();
		QHBoxLayout *layout = new QHBoxLayout();
		layout->setMargin(0);//一定要有
		layout->addWidget(box);
		layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
		widget->setLayout(layout);
		box->setChecked(curdata.isShow);
		connect(box, &QCheckBox::stateChanged, [=](int index) {

			if (index == 0)
			{
				m_colorData[rowCount].isShow = false;
			}
			else
			{
				m_colorData[rowCount].isShow = true;
			}
			emit dataChanged();
		});
        ui->tableWidget->setCellWidget(rowCount, 0, widget);

        QTableWidgetItem* item1 = new QTableWidgetItem();
		item1->setFlags(item1->flags() & ~Qt::ItemIsEditable);
        item1->setData(Qt::EditRole, QString::number(curdata.id));
		item1->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(rowCount,1, item1);

        QTableWidgetItem* item2 = new QTableWidgetItem();
        item2->setData(Qt::EditRole, curdata.name);
        ui->tableWidget->setItem(rowCount,2, item2);
		if (defaultData.find(curdata.id) != defaultData.end())
		{
			item2->setFlags(item2->flags() & ~Qt::ItemIsEditable);
		}
		item2->setTextAlignment(Qt::AlignCenter);
        QToolButton* btn = new QToolButton();
        SetButtonColor(btn,curdata.color);
		connect(btn, &QToolButton::clicked, [=]() {

			CS::Widgets::FramelessDialog outdlg(MainWindow::TheInstance());
            outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
                "font-size: 16px;\n"
                "padding-left: 5px;\n"
                "color: #F2F2F2;\n"
                "background-color:transparent;\n"
                "line-height: 24px;\n");
			outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
			QColorDialog colordlg(m_colorData[rowCount].color, this);
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
				m_colorData[rowCount].color = newCol;
				SetButtonColor(btn, newCol);
			}
			emit dataChanged();
		});
        ui->tableWidget->setCellWidget(rowCount, 3, btn);
    }
	ui->tableWidget->blockSignals(false);
}

std::vector<ClassficationData> FJClassficationTableWidget::getClassficationColor()
{
    return m_colorData;
}
