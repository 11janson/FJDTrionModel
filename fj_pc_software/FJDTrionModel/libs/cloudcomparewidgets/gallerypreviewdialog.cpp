#include "gallerypreviewdialog.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "cloudcompareutils/icore.h"
#include "cswidgets/titlebar.h"
#include "imagepreview.h"
#include "model/projectmodel.h"
#include "ccNote.h"

using namespace CS;
using namespace CS::Widgets;
using namespace CS::MetahubWidgets;


namespace CS {
    namespace MetahubWidgets {
        class GalleryPreViewDialogPrivate : public  QObject
        {
            Q_OBJECT
        public:
            explicit GalleryPreViewDialogPrivate(CS::MetahubWidgets::GalleryPreViewDialog *qptr);
            virtual ~GalleryPreViewDialogPrivate();

        public:
            /**
            *@brief 更新Model数据到UI层显示
            */
            void updateUIFromModel(void);
        private:
            /**
            *@brief 创建界面
            */
            void createWidget();
            /**
            *@brief 创建连接
            */
            void createConnect(void);
            /**
            *@brief 创建翻译
            */
            void retranslateUi();

        private:
            friend class CS::MetahubWidgets::GalleryPreViewDialog;
            CS::MetahubWidgets::GalleryPreViewDialog *m_qptr = nullptr;
        private:
            CS::MetahubWidgets::ImageView       *m_pCenterImageWidget = nullptr;
        private:
            QPushButton                         *m_pNextButton = nullptr;
            QPushButton                         *m_pPreviousButton = nullptr;
        private:
            ccNote *m_pActivatedNode = nullptr;
        };
       
    }
}

GalleryPreViewDialogPrivate::GalleryPreViewDialogPrivate(
    CS::MetahubWidgets::GalleryPreViewDialog *qptr)
{
    m_qptr = qptr;
}
GalleryPreViewDialogPrivate::~GalleryPreViewDialogPrivate()
{

}



void GalleryPreViewDialogPrivate::updateUIFromModel(void)
{
}

void GalleryPreViewDialogPrivate::createWidget()
{
    m_qptr->setAttribute(Qt::WA_DeleteOnClose, true);
    m_qptr->bottomWidget()->setVisible(false);
 
   /* QString strStyleSheet = CS::Core::ICore::themeStyleSheet();
    QFile styleSheet(strStyleSheet);
    if (styleSheet.open(QIODevice::ReadOnly)) {
        m_qptr->getTitleBar()->GetButtonByType(TitleBar::PredefinedButtonType::CloseButton)->setStyleSheet(styleSheet.readAll());
    }

    QHBoxLayout *pMainHLayout = new QHBoxLayout();
    pMainHLayout->setContentsMargins(0, 0, 0, 0);
    m_pNextButton = new QPushButton();
    m_pNextButton->setVisible(false);
    m_pNextButton->setFixedSize(QSize(32, 48));
    m_pNextButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPreviousButton = new QPushButton();
    m_pPreviousButton->setVisible(false);
    m_pPreviousButton->setFixedSize(QSize(32, 48));
    m_pPreviousButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    pMainHLayout->addWidget(m_pPreviousButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
    pMainHLayout->addStretch();
    pMainHLayout->addWidget(m_pNextButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
    m_qptr->getContentHolder()->setLayout(pMainHLayout);*/
    return;
}

void GalleryPreViewDialogPrivate::createConnect(void)
{
}

void GalleryPreViewDialogPrivate::retranslateUi()
{
}











GalleryPreViewDialog::GalleryPreViewDialog(QWidget *parent
    , Qt::WindowFlags f)
    :MetahubFramelessDialog(parent, f)
    , m_dptr(new CS::MetahubWidgets::GalleryPreViewDialogPrivate(this))
{


    m_dptr->createWidget();
    m_dptr->createConnect();
    m_dptr->retranslateUi();

}


void GalleryPreViewDialog::updateUIFromModel(void)
{
    m_dptr->m_pActivatedNode = CS::Model::ProjectModel::instance()->getActivatedNote();

    //[!].更新数据
    m_dptr->updateUIFromModel();

}


GalleryPreViewDialog::~GalleryPreViewDialog()
{
    if (nullptr != m_dptr) {
        delete m_dptr;
    }
    m_dptr = nullptr;
}



#include "gallerypreviewdialog.moc"