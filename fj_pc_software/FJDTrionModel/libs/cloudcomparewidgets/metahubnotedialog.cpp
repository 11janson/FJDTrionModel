#include "metahubnotedialog.h"
#include <QDialog>
#include <QWidget>
#include <QVariant>
#include <QApplication>
#include <QLabel>
#include <QFile>
#include <QTextEdit>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QProcess>
#include <QToolButton>
#include<QPushButton>
#include<QDebug>
#include<QFileDialog>
#include<QListWidget>
#include <QTextEdit>
#include <QButtonGroup>
#include <QScrollArea>
#include <QTimer>
#include <QMainWindow>
#include "imagepreview.h"

#include "cswidgets/flowlayout.h"
#include "cswidgets/framelessfiledialog.h"
#include "cswidgets/titlebar.h"
#include "cswidgets/framelesshelper.h"
#include "cloudcompareutils/icore.h"
#include "model/projectmodel.h"
#include "modelcontrol/cchobjectcontrol.h"
#include "gallerybutton.h"
#include<QStandardPaths>

#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"

#include "ccNote.h"

using namespace CS::MetahubWidgets;
using namespace CS::Widgets;
using namespace CS::Core;
namespace CS::MetahubWidgets {
		class MetahubNoteDialogPrivate : public  QObject
		{
			Q_OBJECT
		public:
			explicit MetahubNoteDialogPrivate(CS::MetahubWidgets::MetahubNoteDialog *qptr);
			virtual ~MetahubNoteDialogPrivate();

        public:
           /**
           *@brief 更新Model数据到UI层显示
           */
            void updateUIFromModel(void);
		private:
			/**
			*@brief 创建界面
			*/
			void createWidget(QWidget *parent);
			/**
			*@brief 创建连接
			*/
			void createConnect(void);
			/**
			*@brief 创建翻译
			*/
			void retranslateUi();

        private:
            /**
            *@brief 清理界面
            */
            void clearWidget(void);
            /**
            *@brief 创建缓存图片
            */
            void createCacheWidget(void);

            /**
            *@breif 更新图库到UI层
            */
            void updateGalleryUIFromModel(void);


            /**
            *@brief 更新UI层数据到Model
            */
            void updateGalleryModelFromUI(void);


            /**
            *@brief 打开图库预览
            */
            void openGalleryPreview(const QImage &image);

            /**
            *@brief 添加图库
            */
            void appendGallery(void);

            /**
           *@brief 更新窗口大小
           */
            void updateWidgetSize(void);


            /**
            *@brief 延迟刷新UI
            */
            void updateDelayedRefreshLabel(QLabel *pLabel);
        public slots:
            /**
            *@brief UI确定按钮
            */
            void slotUIButtonOk(void);

            /**
            *@brief 图库按钮
            */
            void slotGalleryButtonClicked(void);

            /**
            *@brief 图库删除键
            */
            void slotGalleryDeleteClicked(void);

            /**
            *@brief 详细信息文本发生变化
            */
            void slotDetailsTextChanged(void);

          
		private:
			QFrame		    *m_pFrame = nullptr;
			TitleBar        *m_pTitleBar = nullptr;
			QLabel          *m_pTitleLabel = nullptr;
            QLabel          *m_pMarkLabel = nullptr;
            QLabel          *m_pTitleInfoLabel = nullptr;
			QLineEdit       *m_pTitleEdit = nullptr;
			QLabel          *m_pDetailsLabel = nullptr;
			QTextEdit       *m_pDetailsEdit = nullptr;
            QLabel          *m_pPictureLabel = nullptr;
            QLabel          *m_pPictureCountLabel = nullptr;
            QLabel          *m_pPictureInfo = nullptr;
            
		

		private:
			friend class CS::MetahubWidgets::MetahubNoteDialog;
			CS::MetahubWidgets::MetahubNoteDialog *m_qptr = nullptr;
        private:
            int m_nMaxCount = 19;
            ccNote *m_pActivatedNode = nullptr;
        private:
            CS::Widgets::FlowLayout*	m_pNavigationFlowlayout = nullptr;
            QButtonGroup*				m_pButtonGroup = nullptr;
            QScrollArea*                m_pContentScrllArea = nullptr;
		};

      

}
MetahubNoteDialogPrivate::MetahubNoteDialogPrivate(
	CS::MetahubWidgets::MetahubNoteDialog *qptr)
{
	m_qptr = qptr;
}
MetahubNoteDialogPrivate::~MetahubNoteDialogPrivate()
{

}

void MetahubNoteDialogPrivate::createWidget(QWidget *parent)
{

    m_qptr->setAttribute(Qt::WA_DeleteOnClose, true);
	m_qptr->setFixedSize(560, 452);
	//标题栏
	m_qptr->getTitleBar()->setTitleButtonVisible(TitleBar::PredefinedButtonType::LogoButton, false);


    QVBoxLayout *pMainVLayout = new QVBoxLayout();
    pMainVLayout->setContentsMargins(15, 15, 15, 0);
    pMainVLayout->setSpacing(0);

    //[!].标题
    QHBoxLayout *pTitleTopLayout = new QHBoxLayout();
    pTitleTopLayout->setContentsMargins(0, 0, 0, 0);
    pTitleTopLayout->setSpacing(0);
    m_pTitleLabel = new QLabel();
    m_pTitleLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    //m_pTitleLabel->setFixedWidth(30);
    pTitleTopLayout->addWidget(m_pTitleLabel, 0, Qt::AlignLeft);
    pTitleTopLayout->addSpacing(3);
    m_pMarkLabel = new QLabel();
    m_pMarkLabel->setStyleSheet("color: red;");
    m_pMarkLabel->setAlignment(Qt::AlignLeft);
    m_pMarkLabel->setFixedWidth(12);
    m_pMarkLabel->setText("*");
    pTitleTopLayout->addWidget(m_pMarkLabel, 0, Qt::AlignLeft);
    pTitleTopLayout->addStretch();

    m_pTitleInfoLabel = new QLabel();
    m_pTitleInfoLabel->setStyleSheet("color: red;");
    m_pTitleInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_pTitleInfoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pTitleTopLayout->addWidget(m_pTitleInfoLabel, 0, Qt::AlignLeft);
    pMainVLayout->addLayout(pTitleTopLayout);
    pMainVLayout->addSpacing(5);
    m_pTitleEdit = new QLineEdit();
    m_pTitleEdit->setFixedSize(476, 80);
    m_pTitleEdit->setStyleSheet("border: 1px solid #989898;\n"
                                "border - radius: 2px;\n");
    m_pTitleEdit->setFixedHeight(30);
    m_pTitleEdit->setMaxLength(30);
    m_pTitleEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pMainVLayout->addWidget(m_pTitleEdit);

    //[!].详情
    pMainVLayout->addSpacing(13);
    m_pDetailsLabel = new QLabel;
    m_pDetailsLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pMainVLayout->addWidget(m_pDetailsLabel);
    pMainVLayout->addSpacing(5);
    m_pDetailsEdit = new QTextEdit();
    m_pDetailsEdit->setObjectName("DetailsEdit");
    m_pDetailsEdit->setStyleSheet(" QTextEdit#DetailsEdit{border: 1px solid #989898;\n"
                                    "border - radius: 2px;}\n");
    m_pDetailsEdit->setFixedSize(476,80);
    m_pDetailsEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pMainVLayout->addWidget(m_pDetailsEdit);

	/*图片缩略图*/
    QHBoxLayout *pPictureLayout = new QHBoxLayout();
    pPictureLayout->setContentsMargins(0, 12, 0, 0);
    m_pPictureLabel = new QLabel();
    m_pPictureLabel->setFixedSize(150, 20);
    m_pPictureLabel->setAlignment(Qt::AlignLeft);
    m_pPictureLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    pPictureLayout->addWidget(m_pPictureLabel, 0, Qt::AlignLeft);

    m_pPictureInfo = new QLabel();
    m_pPictureInfo->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    m_pPictureInfo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_pPictureInfo->setStyleSheet("color: red;");
    pPictureLayout->addWidget(m_pPictureInfo,0,Qt::AlignCenter);


    m_pPictureCountLabel = new QLabel();
    m_pPictureCountLabel->setAlignment(Qt::AlignRight);
    m_pPictureCountLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPictureCountLabel->setText(QString("0/%1").arg(m_nMaxCount));
    m_pPictureCountLabel->setFixedWidth(50);
    pPictureLayout->addWidget(m_pPictureCountLabel, 0, Qt::AlignRight);
    pMainVLayout->addLayout(pPictureLayout);

    //[!].图片预览列表
    m_pContentScrllArea= new QScrollArea();
    m_pContentScrllArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_pContentScrllArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_pContentScrllArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_pContentScrllArea->setWidgetResizable(true);
    QFrame *pFrame = new QFrame(m_pContentScrllArea);
    pFrame->setStyleSheet("QFrame{padding-left:15px;padding-right:15px;padding-top:0px;padding-bottom:0px;background-color:transparent;}");
    m_pContentScrllArea->setStyleSheet("QScrollArea{border:none; background-color:transparent;}");
    m_pContentScrllArea->setWidget(pFrame);
    m_pNavigationFlowlayout = new CS::Widgets::FlowLayout(pFrame, 0, 12, 12);
    m_pButtonGroup = new QButtonGroup();
    m_pButtonGroup->setExclusive(true);

    QVBoxLayout *pEndVBoxLayout = new QVBoxLayout();
   
    pEndVBoxLayout->setContentsMargins(0, 0, 0, 0);
    pEndVBoxLayout->addLayout(pMainVLayout);
    pEndVBoxLayout->setSpacing(3);
    pEndVBoxLayout->addWidget(m_pContentScrllArea, 0, Qt::AlignTop);
	m_qptr->setLayout(pEndVBoxLayout);

	
}
void MetahubNoteDialogPrivate::createConnect()
{
    //[!].确定按钮
   QPushButton *pButtonOK =  m_qptr->getOKButton();
   connect(pButtonOK, &QPushButton::clicked,
       this, &MetahubNoteDialogPrivate::slotUIButtonOk);


   connect(m_pDetailsEdit, &QTextEdit::textChanged,
       this, &MetahubNoteDialogPrivate::slotDetailsTextChanged);

}
void MetahubNoteDialogPrivate::retranslateUi()
{
	m_pDetailsLabel->setText(tr("Details"));
	m_pTitleLabel->setText(tr("Title"));
    QString str = m_pTitleEdit->text();
    
    
    //[!].增加提示语
    m_pTitleEdit->setPlaceholderText(tr(" Enter here."));
    m_pDetailsEdit->setPlaceholderText(tr(" Enter here."));
    m_pPictureLabel->setText(tr("Add image"));
    return;
}

void MetahubNoteDialogPrivate::updateUIFromModel(void)
{ 
    if (!m_pActivatedNode){

        clearWidget();
        return;
    }
    //[!].获取标题
    m_pTitleEdit->setText(m_pActivatedNode->getTitle());
    m_pDetailsEdit->setText(m_pActivatedNode->getContent());


    //[!].修改对话框名称
    bool bAdd = m_pActivatedNode->getMetaData("notetypeadd").toBool();
    if (bAdd)
    {
        m_qptr->getTitleBar()->setTitle(tr("Add Notes"));
    }
    else
    {
        m_qptr->getTitleBar()->setTitle(tr("Annotation Details"));
    }

    updateWidgetSize();

    //[!].更新图库层数据到UI层
    QTimer::singleShot(30, this, [=]() {
        updateGalleryUIFromModel();

    });
   
  
    return;

}

void MetahubNoteDialogPrivate::slotUIButtonOk(void)
{

    if (!m_pActivatedNode) {
        qDebug() << "activate is empty!";
        return;
    }

    //[!].获取当前选择的实体类型
    CS::Model::ProjectModel *pProjectModel = CS::Model::ProjectModel::instance();
    QString strTitle = m_pTitleEdit->text();
    if (strTitle.isEmpty()) {
        //[!].标题不能为空请从新输入
        m_pTitleEdit->setPlaceholderText(tr(" Title required."));
        m_pTitleInfoLabel->setText(tr(" Title required."));
        m_pTitleEdit->setStyleSheet("border: 1px solid #FF4746;\n"
                                    "border - radius: 2px;\n");
        updateDelayedRefreshLabel(m_pTitleInfoLabel);
        QTimer::singleShot(5 * 1000, this, [=]() {
            m_pTitleEdit->setStyleSheet("border: 1px solid #989898;\n"
                                        "border - radius: 2px;\n");
        });
        return;
    }

    //[!].更新数据到Model层 标题
    m_pActivatedNode->setTitle(strTitle);
    //[!].内容
    QString strContent = m_pDetailsEdit->toPlainText();
    m_pActivatedNode->setContent(strContent);

 
    //[!].判断类型
    bool bAdd = m_pActivatedNode->getMetaData("notetypeadd").toBool();
    if (bAdd){//[!].添加

        std::vector<ccHObject*> pCurentSelectEntitys = pProjectModel->getSelectedEntitys();
        if (!pCurentSelectEntitys.empty()) {
            ccHObject* pEntity = pCurentSelectEntitys.at(0);
            CS::ModelControl::CCHObjectControl control(pEntity);
            //[!].创建注释名称
            QString strName = control.createChildNodeName("Note");
            m_pActivatedNode->setName(strName);
			m_pActivatedNode->setMetaData("expandnode", true);
            pEntity->addChild(m_pActivatedNode);
            m_pActivatedNode->setMetaData("notetypeadd", false);
            emit CS::Model::ProjectModel::instance()->signalRenewalDBTreeModel();
        }

    }
    //[!].刷新状态
    emit CS::Model::ProjectModel::instance()->signalRedrawActiveWindow();
    m_qptr->accept();
}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::clearWidget(void)
{
    m_pTitleEdit->clear();
    m_pDetailsEdit->clear();
    return;
}


void CS::MetahubWidgets::MetahubNoteDialogPrivate::createCacheWidget(void)
{

    GalleryButton* pButton = new CS::MetahubWidgets::GalleryButton();
    pButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    GalleryButton::GalleryButtonType enType = GalleryButton::Add;
    pButton->setGalleryButtonType(enType);
    pButton->updateUIFromModel();
    m_pButtonGroup->addButton(pButton, 0);
    m_pNavigationFlowlayout->addWidget(pButton);
    pButton->setProperty("gallerybutton", true);


    connect(pButton, &QToolButton::clicked, this, &MetahubNoteDialogPrivate::slotGalleryButtonClicked);
    return;
}

void MetahubNoteDialogPrivate::updateGalleryUIFromModel(void)
{
    //[!].按需加载
    if (!m_pActivatedNode) {
        return;
    }
    std::vector<QImage> listImage = m_pActivatedNode->getNoteImages();
    int index = 1;
    for (auto it = listImage.begin(); it != listImage.end(); it++){
        CS::MetahubWidgets::GalleryButton *pGalleryButton = dynamic_cast<GalleryButton*>(m_pButtonGroup->button(index));

        if (!pGalleryButton) {
            //[!].没有就创建
            pGalleryButton = new CS::MetahubWidgets::GalleryButton();
            pGalleryButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            GalleryButton::GalleryButtonType enType = GalleryButton::Thumbnail;
            m_pButtonGroup->addButton(pGalleryButton, index);
            pGalleryButton->setGalleryButtonType(enType);
            pGalleryButton->updateUIFromModel();
            pGalleryButton->setProperty("gallerybutton", true);
            m_pNavigationFlowlayout->addWidget(pGalleryButton);
            connect(pGalleryButton, &QPushButton::clicked, this, &MetahubNoteDialogPrivate::slotGalleryButtonClicked);
            connect(pGalleryButton, &GalleryButton::signalDeleteGalleryButton, this, &MetahubNoteDialogPrivate::slotGalleryDeleteClicked);
        }

        pGalleryButton->setGallery(*it);
        pGalleryButton->setProperty("index", index);
        pGalleryButton->updateUIFromModel();
        index++;

    }
    for (int i = index; i < m_pButtonGroup->buttons().size(); i++){

        QAbstractButton *pButton = m_pButtonGroup->button(i);
        if (pButton){
            m_pButtonGroup->removeButton(pButton);
            pButton->deleteLater();
        }
    }

    QString strCount = QString("%1/%2").arg(listImage.size()).arg(m_nMaxCount);
    m_pPictureCountLabel->setText(strCount);

    //[!]
    updateWidgetSize();
    return;
}

void MetahubNoteDialogPrivate::updateGalleryModelFromUI(void)
{
    if (!m_pActivatedNode) {
        return;
    }
    
    std::vector<QImage> listImage;
    for (auto pButton : m_pButtonGroup->buttons()){
        CS::MetahubWidgets::GalleryButton *pGalleryButton = dynamic_cast<GalleryButton*>(pButton);
        if (!pGalleryButton) {
            continue;
        }
        if (!pGalleryButton->isVisible() || 
            pGalleryButton->getGalleryButtonType() == GalleryButton::Add){
            continue;
        }
        QImage image = pGalleryButton->getGallery();
        if (image.isNull()){
            continue;
        }
        listImage.push_back(image);
    }
    m_pActivatedNode->setNoteImages(listImage);
    QString strCount = QString("%1/%2").arg(listImage.size()).arg(m_nMaxCount);
    m_pPictureCountLabel->setText(strCount);
    return;
}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::openGalleryPreview(const QImage & image)
{
    if (!m_pActivatedNode) {
        return;
    }

    CS::MetahubWidgets::ImageView *pImageView = new CS::MetahubWidgets::ImageView();
    QMainWindow *pMainWindow = CS::Model::ProjectModel::instance()->getMainWindow();
    int index = m_pButtonGroup->checkedButton()->property("index").toInt();
    pImageView->setImageList(m_pActivatedNode->getNoteImages());
    pImageView->setActivateIndex(index);
    pImageView->updateUIFromModel();
    if (pMainWindow) {
        QPoint point = pMainWindow->pos();
        QRect rect = pMainWindow->rect();
        pImageView->setFixedSize(QSize(rect.width(), rect.height()));
        pImageView->move(point); 
    }
    pImageView->exec();
}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::appendGallery(void)
{
    if (!m_pActivatedNode) {
        return;
    }

    QString strDefaultPath = CS::Model::ProjectModel::instance()->property("defalutpath").toString();
    if (strDefaultPath.isEmpty()) {
        strDefaultPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    }
    QStringList listFileName = CS::Widgets::FramelessFileDialog::getOpenFileNames(m_qptr, tr("Add picture"), strDefaultPath, tr("(*.jpg *.png *.bmp *.tif)"));
    if (listFileName.isEmpty()) {

        return;
    }

    QFileInfo file(listFileName.at(0));
    strDefaultPath = file.absoluteDir().path();
    CS::Model::ProjectModel::instance()->setProperty("defalutpath", strDefaultPath);

    for (int i = 0; i < listFileName.size(); i++) {
        QImage image;
        if (!image.load(listFileName.at(i))) {
            break;
        }

        if (!m_pActivatedNode->appendImage(image)) {
            m_pPictureInfo->setText(tr("A maximum of 19 pictures allowed."));
            updateDelayedRefreshLabel(m_pPictureInfo);
            break;
        }
    }

    QTimer::singleShot(20, this, [=]() {
        updateGalleryUIFromModel();
        updateWidgetSize();
    });


    return;

}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::updateWidgetSize(void)
{
    int nMainH = 460;
    int nOffset = 98;
    int nScrollArea = 104;

    int nCount= m_pActivatedNode->getNoteImages().size() + 1;
    int nRowCount = (nCount / 5);
    if (nRowCount >= 4) {
        nRowCount = 3;
        nOffset = 98;
    } 

    int iH = nMainH + nOffset * nRowCount;
    int iAreaH = nScrollArea + nOffset * nRowCount;
    m_pContentScrllArea->setFixedHeight(iAreaH);
    m_qptr->setFixedHeight(iH - 10); 
}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::updateDelayedRefreshLabel(QLabel * pLabel)
{
    QTimer::singleShot(5 * 1000, this, [=]() {
        if (pLabel) {
            pLabel->setText("");
       }
    });
}

void MetahubNoteDialogPrivate::slotGalleryButtonClicked(void)
{
    if (!m_pActivatedNode) {
        return;
    }

    //[!].获取对象
    CS::MetahubWidgets::GalleryButton *pGalleryButton = dynamic_cast<GalleryButton*>(this->sender());
    if (!pGalleryButton) {
        return;
    }

    //[!].判断类型
    GalleryButton::GalleryButtonType enType = pGalleryButton->getGalleryButtonType();
    if (enType == GalleryButton::GalleryButtonType::Add) {
        //[!].添加
        int isize = m_pActivatedNode->getNoteImages().size();
        if (isize < m_nMaxCount) {

            appendGallery();
        }
        else {
            m_pPictureInfo->setText(tr("Add a maximum of 19 photos"));
            updateDelayedRefreshLabel(m_pPictureInfo);
        }
    }
    else {
        //[!].预览
        QImage image = pGalleryButton->getGallery();
        openGalleryPreview(image);
        return;
    }

    return;
}

void MetahubNoteDialogPrivate::slotGalleryDeleteClicked(void)
{
    //[!].获取对象
    CS::MetahubWidgets::GalleryButton *pGalleryButton = dynamic_cast<GalleryButton*>(this->sender());
    if (!pGalleryButton) {
        return;
    }
    m_pButtonGroup->removeButton(pGalleryButton);
    pGalleryButton->deleteLater();

    //[!].更新UI数据到model
    updateGalleryModelFromUI();

    updateWidgetSize();
    return;
}

void CS::MetahubWidgets::MetahubNoteDialogPrivate::slotDetailsTextChanged(void)
{
    QString textContent = m_pDetailsEdit->toPlainText();
    int nLength = textContent.count();
    m_pDetailsEdit->blockSignals(true);
    int nMaxCount = 1000;
    if (nLength > nMaxCount) {
        int position = m_pDetailsEdit->textCursor().position();
        QTextCursor textCursor = m_pDetailsEdit->textCursor();
        textContent.remove(position - (nLength - nMaxCount), nLength - nMaxCount);
        m_pDetailsEdit->setText(textContent);
        textCursor.setPosition(position - (nLength - m_nMaxCount));
        m_pDetailsEdit->setTextCursor(textCursor);
    }
    m_pDetailsEdit->blockSignals(false);
}

MetahubNoteDialog::MetahubNoteDialog(QWidget *parent
	, Qt::WindowFlags f)
	:MetahubFramelessDialog(parent, f)
	, m_dptr(new CS::MetahubWidgets::MetahubNoteDialogPrivate(this))
{
	

	m_dptr->createWidget(parent);
	m_dptr->createConnect();
    m_dptr->createCacheWidget();
	m_dptr->retranslateUi();
	
}


void MetahubNoteDialog::updateUIFromModel(void)
{
    m_dptr->m_pActivatedNode = CS::Model::ProjectModel::instance()->getActivatedNote();

    //[!].更新数据
    m_dptr->updateUIFromModel();
   
}

void MetahubNoteDialog::slotUIButtonOk(void)
{

    return;
}

MetahubNoteDialog::~MetahubNoteDialog()
{
	if (nullptr != m_dptr) {
		delete m_dptr;
	}
	m_dptr = nullptr;
}


#include "metahubnotedialog.moc"