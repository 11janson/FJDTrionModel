#include "gallerybutton.h"
#include <qvariant.h>
#include <QLayout>
#include <QImage>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "cloudcompareutils/icore.h"
using namespace CS::Core;


using namespace CS;
using namespace CS::MetahubWidgets;

namespace CS {
    namespace MetahubWidgets {
        class GalleryButtonPrivate : public QObject
        {
            Q_OBJECT
        public:
            explicit GalleryButtonPrivate(GalleryButton *qptr);
        public:
            /**
            *@brief 更新数据到UI层
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
            void createConnects();

            /**
            * @brief 实时翻译时，刷新UI文字显示
            */
            void retranslateUi();

        private:
            CS::MetahubWidgets::GalleryButton::GalleryButtonType m_enGalleryType = GalleryButton::Thumbnail;
            QImage  m_pThumbnailImage;
        private:
            friend class GalleryButton;
            CS::MetahubWidgets::GalleryButton *m_pQptr = nullptr;

        private:
            QLabel *m_pBottomLabel = nullptr;
            QPushButton *m_pDeleteButton = nullptr;
            QSize m_size = QSize(86, 86);
            QPushButton *m_pAddIconButton = nullptr;
            QVBoxLayout *m_pMainVlayout = nullptr;

        };
    }
}

GalleryButtonPrivate::GalleryButtonPrivate(GalleryButton *qptr)
{
    m_pQptr = qptr;
}

void GalleryButtonPrivate::createWidget()
{
    m_pMainVlayout = new QVBoxLayout();
    m_pMainVlayout->setContentsMargins(1, 1, 1, 1);

    QHBoxLayout *pDeleteLayout = new QHBoxLayout();
    pDeleteLayout->setContentsMargins(0, 0, 0, 0);
    pDeleteLayout->setSpacing(0);

    m_pDeleteButton = new QPushButton(m_pQptr);
    m_pDeleteButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pDeleteButton->setFixedSize(QSize(24, 24));
    m_pDeleteButton->setIconSize(QSize(24, 24));
    m_pDeleteButton->setVisible(false);
    m_pDeleteButton->setCheckable(true);
    pDeleteLayout->addStretch();
    pDeleteLayout->addWidget(m_pDeleteButton, 0, Qt::AlignRight);
    m_pMainVlayout->addLayout(pDeleteLayout);
    m_pMainVlayout->addStretch();
    m_pDeleteButton->setIcon(ICore::resourceThemeImage("noteremoval.png"));

    m_pAddIconButton = new QPushButton(m_pQptr);
    m_pAddIconButton->setCheckable(false);
    m_pAddIconButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pAddIconButton->setVisible(false);
    m_pMainVlayout->addWidget(m_pAddIconButton, 0, Qt::AlignHCenter | Qt::AlignVCenter);
    m_pMainVlayout->addSpacing(10);
    m_pBottomLabel = new QLabel(m_pQptr);
    m_pBottomLabel->setAlignment(Qt::AlignCenter);
    m_pBottomLabel->setFixedHeight(16);
    m_pBottomLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_pBottomLabel->setVisible(false);
    m_pMainVlayout->addWidget(m_pBottomLabel, 0, Qt::AlignBottom | Qt::AlignHCenter);
    m_pQptr->setLayout(m_pMainVlayout);
    m_pQptr->setCheckable(true);

    m_pDeleteButton->setStyleSheet("background-color:transparent;border:none;");
    m_pAddIconButton->setStyleSheet("background-color:transparent;border:none;");
    m_pAddIconButton->setAttribute(Qt::WA_TransparentForMouseEvents, true);
   
}

void GalleryButtonPrivate::createConnects()
{
    connect(m_pDeleteButton, &QPushButton::clicked,
        m_pQptr, &GalleryButton::signalDeleteGalleryButton);
}

void CS::MetahubWidgets::GalleryButtonPrivate::retranslateUi()
{
    //m_pBottomLabel->setText(tr("Enlarge"));
    //m_pBottomLabel->setStyleSheet("background: rgba(0,0,0,0.6);");
}


void GalleryButtonPrivate::updateUIFromModel(void)
{
    if (m_enGalleryType == GalleryButton::Add){
        //[!].添加按钮
        m_pBottomLabel->setText(tr("add"));
        m_pAddIconButton->setIcon(ICore::resourceThemeImage("add_2x.png"));
      
        m_pAddIconButton->setFixedSize(QSize(24, 24));
        m_pAddIconButton->setIconSize(QSize(24, 24));
        m_pAddIconButton->setVisible(true);
        m_pBottomLabel->setVisible(true);
        m_pBottomLabel->setStyleSheet("border - radius: 1px;\n"
                                      "border: 1px solid #989898; ");
        m_pMainVlayout->setContentsMargins(1, 18, 1, 18);
    }
    else{
        if (!m_pThumbnailImage.isNull()) {
            QImage image = m_pThumbnailImage.scaled(m_size.width(), m_size.height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            QPixmap pix = QPixmap::fromImage(image);
            if (!pix.isNull()) {
                m_pQptr->setIconSize(QSize(75, 75));
                m_pQptr->setIcon(pix);
            }
        }
       
        
    }
}



GalleryButton::GalleryButton(QWidget *parent)
    : QPushButton(parent)
    , m_pDptr(new CS::MetahubWidgets::GalleryButtonPrivate(this))
{

    setIconSize(m_pDptr->m_size);
    setFixedSize(m_pDptr->m_size);
    //setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
   


    m_pDptr->createWidget();
    m_pDptr->createConnects();
    m_pDptr->retranslateUi();
    return;
}

void CS::MetahubWidgets::GalleryButton::updateUIFromModel(void)
{
    m_pDptr->updateUIFromModel();
}

void CS::MetahubWidgets::GalleryButton::setGallery(const QImage & image)
{
    m_pDptr->m_pThumbnailImage = image;
}

QImage GalleryButton::getGallery(void)
{
    return m_pDptr->m_pThumbnailImage;
}

CS::MetahubWidgets::GalleryButton::GalleryButtonType GalleryButton::getGalleryButtonType(void)
{
    return m_pDptr->m_enGalleryType;
}

void CS::MetahubWidgets::GalleryButton::setGalleryButtonType(GalleryButtonType enType)
{
    m_pDptr->m_enGalleryType = enType;
}

void CS::MetahubWidgets::GalleryButton::retranslateUi()
{
    m_pDptr->retranslateUi();
}

void GalleryButton::enterEvent(QEvent *event)
{
    QPushButton::enterEvent(event);
    if (m_pDptr->m_enGalleryType == GalleryButton::Thumbnail){
        m_pDptr->m_pBottomLabel->setVisible(true);
        m_pDptr->m_pDeleteButton->setVisible(true);
        this->setCursor(Qt::PointingHandCursor);
    }
    
}
void GalleryButton::leaveEvent(QEvent *event)
{
    QPushButton::leaveEvent(event);
    if (m_pDptr->m_enGalleryType == GalleryButton::Thumbnail) {
        m_pDptr->m_pBottomLabel->setVisible(false);
        m_pDptr->m_pDeleteButton->setVisible(false);
        this->setCursor(Qt::ArrowCursor);
    }
   
}

#include "gallerybutton.moc"

