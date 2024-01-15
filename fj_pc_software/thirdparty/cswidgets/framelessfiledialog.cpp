#include "framelessfiledialog.h"
#include"framelessmessagebox.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QDebug>
#include <QLabel>
#include <QCoreApplication>
using namespace CS::Widgets;

Q_GLOBAL_STATIC(QUrl, lastVisitedDir)

//FileDialog::FileDialog(QWidget *parent) : QDialog(parent)
//{
//    QHBoxLayout* playout = new QHBoxLayout;

//    QFileDialog* dlg = new QFileDialog(this);
//    dlg->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    dlg->setWindowFlags((dlg->windowFlags() | Qt::Widget) & (~Qt::Dialog));
//    playout->addWidget(dlg);

//    setLayout(playout);
//}

inline static QUrl _qt_get_directory(const QUrl &url)
{
    if (url.isLocalFile()) {
        QFileInfo info = QFileInfo(QDir::current(), url.toLocalFile());
        if (info.exists() && info.isDir())
            return QUrl::fromLocalFile(QDir::cleanPath(info.absoluteFilePath()));
        info.setFile(info.absolutePath());
        if (info.exists() && info.isDir())
            return QUrl::fromLocalFile(info.absoluteFilePath());
        return QUrl();
    } else {
        return url;
    }
}

FramelessFileDialog::FramelessFileDialog(QWidget *parent, Qt::WindowFlags f)
    : FramelessDialog(parent)
{
    QVBoxLayout* playout = new QVBoxLayout;
    playout->setMargin(0);
    playout->setSpacing(0);

    m_pFileDialog = new QFileDialog(this, f);
    m_pFileDialog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_pFileDialog->setWindowFlags((m_pFileDialog->windowFlags() | Qt::Widget) & (~Qt::Dialog));
    playout->addWidget(m_pFileDialog);

	//m_pFileDialog->setLabelText(QFileDialog::Accept, QCoreApplication::translate("FramelessFileDialog", "Open", nullptr));
	m_pFileDialog->setLabelText(QFileDialog::Reject, QCoreApplication::translate("FramelessFileDialog", "Cancel", nullptr));
	bottomlabel = new QLabel(this);
	bottomlabel->setFixedHeight(10);
	bottomlabel->setObjectName("filedialogbottomlabel");
	playout->addWidget(bottomlabel);
    setLayout(playout);
    bottomWidget()->hide();
    connect(m_pFileDialog, &QFileDialog::rejected, this, &FramelessFileDialog::rejected);
    connect(m_pFileDialog, &QFileDialog::accepted, this, &FramelessFileDialog::accepted);
	
    resize(800,600);
}

FramelessFileDialog::FramelessFileDialog(QWidget *parent, const QString &caption, const QString &directory, const QString &filter)
    : FramelessDialog(parent)
{
    QVBoxLayout* playout = new QVBoxLayout;
    playout->setMargin(0);
    playout->setSpacing(0);
    m_pFileDialog = new QFileDialog(this, caption, directory, filter);
    m_pFileDialog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_pFileDialog->setWindowFlags((m_pFileDialog->windowFlags() | Qt::Widget) & (~Qt::Dialog));
	m_pFileDialog->setViewMode(QFileDialog::Detail);
    playout->addWidget(m_pFileDialog);

	//m_pFileDialog->setLabelText(QFileDialog::Accept, QCoreApplication::translate("FramelessFileDialog", "Open", nullptr));
	//m_pFileDialog->setLabelText(QFileDialog::Reject, QCoreApplication::translate("FramelessFileDialog", "Cancel", nullptr));
 //   m_pFileDialog->setLabelText(QFileDialog::FileType, QCoreApplication::translate("FramelessFileDialog", "File type:", nullptr));


	bottomlabel = new QLabel(this);
	bottomlabel->setFixedHeight(10);
	bottomlabel->setObjectName("filedialogbottomlabel");
	playout->addWidget(bottomlabel);
    setLayout(playout);
    bottomWidget()->hide();

    connect(m_pFileDialog, &QFileDialog::rejected, this, &FramelessFileDialog::rejected);
    connect(m_pFileDialog, &QFileDialog::accepted, this, &FramelessFileDialog::accepted);
    connect(m_pFileDialog, &QFileDialog::rejected, this, &FramelessFileDialog::slot_rejected);
    connect(m_pFileDialog, &QFileDialog::accepted, this, &FramelessFileDialog::slot_accepted);
//    m_pFileDialog->show();
    resize(800,600);
}

void FramelessFileDialog::setDirectory(const QString &directory)
{
    if(m_pFileDialog)
        m_pFileDialog->setDirectory(directory);
}

void FramelessFileDialog::setDirectory(const QDir &directory)
{
    if(m_pFileDialog)
        m_pFileDialog->setDirectory(directory);
}

QDir FramelessFileDialog::directory() const
{
    if(m_pFileDialog)
        return m_pFileDialog->directory();

    return QDir();
}

void FramelessFileDialog::setDirectoryUrl(const QUrl &directory)
{
    if(m_pFileDialog)
        m_pFileDialog->setDirectoryUrl(directory);
}

QUrl FramelessFileDialog::directoryUrl() const
{
    if(m_pFileDialog)
        return m_pFileDialog->directoryUrl();

    return QUrl();
}

void FramelessFileDialog::selectFile(const QString &filename)
{
    if(m_pFileDialog)
        m_pFileDialog->selectFile(filename);
}

QStringList FramelessFileDialog::selectedFiles() const
{
    if(m_pFileDialog)
        return m_pFileDialog->selectedFiles();

    return QStringList();
}

void FramelessFileDialog::selectUrl(const QUrl &url)
{
    if(m_pFileDialog)
        m_pFileDialog->selectUrl(url);
}

QList<QUrl> FramelessFileDialog::selectedUrls() const
{
    if(m_pFileDialog)
        return m_pFileDialog->selectedUrls();

    return QList<QUrl>();
}

void FramelessFileDialog::setNameFilterDetailsVisible(bool enabled)
{
    if(m_pFileDialog)
        m_pFileDialog->setNameFilterDetailsVisible(enabled);
}

bool FramelessFileDialog::isNameFilterDetailsVisible() const
{
    if(m_pFileDialog)
        return m_pFileDialog->isNameFilterDetailsVisible();

    return false;
}

void FramelessFileDialog::setNameFilter(const QString &filter)
{
    if(m_pFileDialog)
        m_pFileDialog->setNameFilter(filter);
}

void FramelessFileDialog::setNameFilters(const QStringList &filters)
{
    if(m_pFileDialog)
        m_pFileDialog->setNameFilters(filters);
}

QStringList FramelessFileDialog::nameFilters() const
{
    if(m_pFileDialog)
        return m_pFileDialog->nameFilters();

    return QStringList();
}

void FramelessFileDialog::selectNameFilter(const QString &filter)
{
    if(m_pFileDialog)
        m_pFileDialog->setNameFilter(filter);
}

//QString FramelessFileDialog::selectedMimeTypeFilter() const
//{
//    if(m_pFileDialog)
//        return m_pFileDialog->selectedMimeTypeFilter();

//    return QString();
//}

QString FramelessFileDialog::selectedNameFilter() const
{
    if(m_pFileDialog)
        return m_pFileDialog->selectedNameFilter();

    return QString();
}

#ifndef QT_NO_MIMETYPE
void FramelessFileDialog::setMimeTypeFilters(const QStringList &filters)
{
    if(m_pFileDialog)
        m_pFileDialog->setMimeTypeFilters(filters);
}

QStringList FramelessFileDialog::mimeTypeFilters() const
{
    if(m_pFileDialog)
        return m_pFileDialog->mimeTypeFilters();

    return QStringList();
}

void FramelessFileDialog::selectMimeTypeFilter(const QString &filter)
{
    if(m_pFileDialog)
        m_pFileDialog->selectMimeTypeFilter(filter);
}
#endif

QDir::Filters FramelessFileDialog::filter() const
{
    if(m_pFileDialog)
        return m_pFileDialog->filter();

    return QDir::Dirs;
}

void FramelessFileDialog::setFilter(QDir::Filters filters)
{
    if(m_pFileDialog)
        m_pFileDialog->setFilter(filters);
}

void FramelessFileDialog::setViewMode(QFileDialog::ViewMode mode)
{
    if(m_pFileDialog)
        m_pFileDialog->setViewMode(mode);
}

QFileDialog::ViewMode FramelessFileDialog::viewMode() const
{
    if(m_pFileDialog)
        return m_pFileDialog->viewMode();

    return QFileDialog::Detail;
}

void FramelessFileDialog::setFileMode(QFileDialog::FileMode mode)
{
    if(m_pFileDialog)
        m_pFileDialog->setFileMode(mode);
}

QFileDialog::FileMode FramelessFileDialog::fileMode() const
{
    if(m_pFileDialog)
        return m_pFileDialog->fileMode();

    return QFileDialog::AnyFile;
}

void FramelessFileDialog::setAcceptMode(QFileDialog::AcceptMode mode)
{
    if(m_pFileDialog)
        m_pFileDialog->setAcceptMode(mode);
}

QFileDialog::AcceptMode FramelessFileDialog::acceptMode() const
{
    if(m_pFileDialog)
        return m_pFileDialog->acceptMode();

    return QFileDialog::AcceptOpen;
}

void FramelessFileDialog::setReadOnly(bool enabled)
{
    if(m_pFileDialog)
        m_pFileDialog->setReadOnly(enabled);
}

bool FramelessFileDialog::isReadOnly() const
{
    if(m_pFileDialog)
        return m_pFileDialog->isReadOnly();

    return false;
}

void FramelessFileDialog::setResolveSymlinks(bool enabled)
{
    if(m_pFileDialog)
        m_pFileDialog->setResolveSymlinks(enabled);
}

bool FramelessFileDialog::resolveSymlinks() const
{
    if(m_pFileDialog)
        return m_pFileDialog->resolveSymlinks();

    return false;
}

void FramelessFileDialog::setSidebarUrls(const QList<QUrl> &urls)
{
    if(m_pFileDialog)
        m_pFileDialog->setSidebarUrls(urls);
}

QList<QUrl> FramelessFileDialog::sidebarUrls() const
{
    if(m_pFileDialog)
        return m_pFileDialog->sidebarUrls();

    return QList<QUrl>();
}

QByteArray FramelessFileDialog::saveState() const
{
    if(m_pFileDialog)
        return m_pFileDialog->saveState();

    return QByteArray();
}

bool FramelessFileDialog::restoreState(const QByteArray &state)
{
    if(m_pFileDialog)
        return m_pFileDialog->restoreState(state);

    return false;
}

void FramelessFileDialog::setConfirmOverwrite(bool enabled)
{
    if(m_pFileDialog)
        m_pFileDialog->setConfirmOverwrite(enabled);
}

bool FramelessFileDialog::confirmOverwrite() const
{
    if(m_pFileDialog)
        return m_pFileDialog->confirmOverwrite();

    return false;
}

void FramelessFileDialog::setDefaultSuffix(const QString &suffix)
{
    if(m_pFileDialog)
        m_pFileDialog->setDefaultSuffix(suffix);
}

QString FramelessFileDialog::defaultSuffix() const
{
    if(m_pFileDialog)
        return m_pFileDialog->defaultSuffix();

    return QString();
}

void FramelessFileDialog::setHistory(const QStringList &paths)
{
    if(m_pFileDialog)
        m_pFileDialog->setHistory(paths);
}

QStringList FramelessFileDialog::history() const
{
    if(m_pFileDialog)
        return m_pFileDialog->supportedSchemes();

    return QStringList();
}

void FramelessFileDialog::setItemDelegate(QAbstractItemDelegate *delegate)
{
    if(m_pFileDialog)
        m_pFileDialog->setItemDelegate(delegate);
}

QAbstractItemDelegate *FramelessFileDialog::itemDelegate() const
{
    if(m_pFileDialog)
        return m_pFileDialog->itemDelegate();

    return nullptr;
}

void FramelessFileDialog::setIconProvider(QFileIconProvider *provider)
{
    if(m_pFileDialog)
        m_pFileDialog->setIconProvider(provider);
}

QFileIconProvider *FramelessFileDialog::iconProvider() const
{
    if(m_pFileDialog)
        return m_pFileDialog->iconProvider();

    return nullptr;
}

void FramelessFileDialog::setLabelText(QFileDialog::DialogLabel label, const QString &text)
{
    if(m_pFileDialog)
        m_pFileDialog->setLabelText(label, text);
}

QString FramelessFileDialog::labelText(QFileDialog::DialogLabel label) const
{
    if(m_pFileDialog)
        return m_pFileDialog->labelText(label);

    return QString();
}

void FramelessFileDialog::setSupportedSchemes(const QStringList &schemes)
{
    if(m_pFileDialog)
        m_pFileDialog->setSupportedSchemes(schemes);
}

QStringList FramelessFileDialog::supportedSchemes() const
{
    if(m_pFileDialog)
        return m_pFileDialog->supportedSchemes();

    return QStringList();
}

#ifndef QT_NO_PROXYMODEL
void FramelessFileDialog::setProxyModel(QAbstractProxyModel *model)
{
    if(m_pFileDialog)
        m_pFileDialog->setProxyModel(model);
}

QAbstractProxyModel *FramelessFileDialog::proxyModel() const
{
    if(m_pFileDialog)
        return m_pFileDialog->proxyModel();

    return nullptr;
}
#endif

void FramelessFileDialog::setOption(QFileDialog::Option option, bool on)
{
    if(m_pFileDialog)
        m_pFileDialog->setOption(option, on);
}

bool FramelessFileDialog::testOption(QFileDialog::Option option) const
{
    if(m_pFileDialog)
        return m_pFileDialog->testOption(option);

    return false;
}

void FramelessFileDialog::setOptions(QFileDialog::Options options)
{
    if(m_pFileDialog)
        m_pFileDialog->setOptions(options);
}

QFileDialog::Options FramelessFileDialog::options() const
{
    if(m_pFileDialog)
        return m_pFileDialog->options();

    return QFileDialog::ShowDirsOnly;
}

//void FileDialog::open(QObject *receiver, const char *member)
//{
//    if(m_pFileDialog)
//        m_pFileDialog->open(receiver, member);
//}

//void FileDialog::setVisible(bool visible)
//{
//    if(m_pFileDialog)
//        m_pFileDialog->setVisible(visible);
//}


QString FramelessFileDialog::getOpenFileName(QWidget *parent, const QString &caption, const QString &dir, const QString &filter,
                                    QString *selectedFilter, QFileDialog::Options options)
{
    const QStringList schemes = QStringList(QStringLiteral("file"));
    const QUrl selectedUrl = getOpenFileUrl(parent, caption, QUrl::fromLocalFile(dir), filter, selectedFilter, options, schemes);
    return selectedUrl.toLocalFile();
}

QUrl FramelessFileDialog::getOpenFileUrl(QWidget *parent, const QString &caption, const QUrl &dir, const QString &filter, QString *selectedFilter, QFileDialog::Options options, const QStringList &supportedSchemes)
{
    FramelessFileDialog* pDialog = new FramelessFileDialog(parent, caption, workingDirectory(dir).toLocalFile(), filter);
    pDialog->setFileMode(QFileDialog::ExistingFile);
    pDialog->setOptions(options);
    pDialog->selectFile(initialSelection(dir));
	pDialog->setWindowTitle(QCoreApplication::translate("FramelessFileDialog", "Open File", nullptr));
    pDialog->setSupportedSchemes(supportedSchemes);
    if (selectedFilter && !selectedFilter->isEmpty())
        pDialog->selectNameFilter(*selectedFilter);
    if (pDialog->exec() == QDialog::Accepted) {
        if (selectedFilter)
            *selectedFilter = pDialog->selectedNameFilter();
        return pDialog->selectedUrls().value(0);
    }

    return QUrl();
}

QString FramelessFileDialog::getSaveFileName(QWidget *parent, const QString &caption, const QString &dir, const QString &filter, QString *selectedFilter, QFileDialog::Options options)
{
    const QStringList schemes = QStringList(QStringLiteral("file"));
    const QUrl selectedUrl = getSaveFileUrl(parent, caption, QUrl::fromLocalFile(dir), filter, selectedFilter, options, schemes);
    return selectedUrl.toLocalFile();
}

QUrl FramelessFileDialog::getSaveFileUrl(QWidget *parent, const QString &caption, const QUrl &dir, const QString &filter, QString *selectedFilter, QFileDialog::Options options, const QStringList &supportedSchemes)
{
//    FileDialogArgs args;
//    args.parent = parent;
//    args.caption = caption;
//    args.directory = workingDirectory(dir);
//    args.selection = initialSelection(dir);
//    args.filter = filter;
//    args.mode = QFileDialog::AnyFile;
//    args.options = options;

    FramelessFileDialog* pDialog = new FramelessFileDialog(parent, caption, workingDirectory(dir).toLocalFile(), filter);
    pDialog->setFileMode(QFileDialog::AnyFile);
    pDialog->setOptions(options);
    pDialog->selectFile(initialSelection(dir));

//    FileDialog dialog(args);
    pDialog->setSupportedSchemes(supportedSchemes);
    pDialog->setAcceptMode(QFileDialog::AcceptSave);
	pDialog->setWindowTitle(QCoreApplication::translate("FramelessFileDialog", "Save As", nullptr));
    //if (selectedFilter && !selectedFilter->isEmpty())
    //    pDialog->selectNameFilter(*selectedFilter);
    if (pDialog->exec() == QDialog::Accepted) {
        if (selectedFilter)
            *selectedFilter = pDialog->selectedNameFilter();
		//[!]对于保存文件特殊字符处理
		bool currentstate = true;
		//[!]用户在输出框中输入的字符串
		QString strName = "";
        QString strUrl = pDialog->selectedUrls().value(0).toString();
		if (strUrl.isEmpty())
		{
			currentstate = false;
		}
		else
		{
			QString tempStrUrl = strUrl;
			if (tempStrUrl.mid(0, 8) == "file:///")
			{
				tempStrUrl.remove(0, 8);
			}
			tempStrUrl = QDir::fromNativeSeparators(tempStrUrl);
			//[!]得到真实存在地址
			QString realpath = "";
			QStringList list = tempStrUrl.split("/");

			if (list.isEmpty())
			{
				currentstate = false;
			}
			for (int i = 0; i < list.size()-1; i++)
			{
				if (list.at(i).isEmpty())
				{
					continue;
				}
				QString tempStr = "";
				tempStr += list.at(i) + "/";
				QDir dirs(realpath + tempStr);
				if (!dirs.exists())
				{
					break;
				}
				realpath += tempStr;
			}
			QDir dir(realpath);
			if (!dir.exists())
			{
				currentstate = false;
			}
			else
			{
				QStringList list = tempStrUrl.split(realpath);
				if (list.size() != 2)
				{
					currentstate = false;
				}
				else
				{
					tempStrUrl = list.at(1);
				}
			}

			//[!]对于文件url后缀带 / \ 处理
			if (tempStrUrl.isEmpty())
			{
				currentstate = false;
			}
			else
			{
				//[!]文件保存时对特殊字符限制并提示
				if (tempStrUrl.contains('*') ||
					tempStrUrl.contains('?') ||
					tempStrUrl.contains('<') ||
					tempStrUrl.contains('>') ||
					tempStrUrl.contains('/') ||
					tempStrUrl.contains('\\'))
				{
					currentstate = false;
				}
				//[!]特殊 | 字符处理
				QStringList namesCheck = tempStrUrl.split('%');
				for (int i = 0; i < namesCheck.size(); i++)
				{
					QString arrays = namesCheck.at(i);
					if (arrays.at(0) == '7' && arrays.at(1) == 'C')
					{
						currentstate = false;
					}
				}
			}

		}

	   if (!currentstate)
	   {
		   CS::Widgets::FramelessMessageBox::critical(nullptr,
			   QCoreApplication::translate("FramelessFileDialog", "Save failed", nullptr),
			   QCoreApplication::translate("FramelessFileDialog", "The file name cannot contain any of the following characters", nullptr)
			   + "'?' '*' '<' '>' '\\' '/' '|'");
		   return QUrl();
	   }
        return pDialog->selectedUrls().value(0);
    }
    return QUrl();
}

QString FramelessFileDialog::getExistingDirectory(QWidget *parent, const QString &caption, const QString &dir, QFileDialog::Options options)
{
    const QStringList schemes = QStringList(QStringLiteral("file"));
    const QUrl selectedUrl = getExistingDirectoryUrl(parent, caption, QUrl::fromLocalFile(dir), options, schemes);
    return selectedUrl.toLocalFile();
}

QUrl FramelessFileDialog::getExistingDirectoryUrl(QWidget *parent, const QString &caption, const QUrl &dir, QFileDialog::Options options, const QStringList &supportedSchemes)
{
//    FileDialogArgs args;
//    args.parent = parent;
//    args.caption = caption;
//    args.directory = workingDirectory(dir);
//    args.mode = (options & QFileDialog::ShowDirsOnly ? QFileDialog::DirectoryOnly : QFileDialog::Directory);
//    args.options = options;

//    FileDialog dialog(args);
    FramelessFileDialog* pDialog = new FramelessFileDialog(parent, caption, workingDirectory(dir).toLocalFile());
    pDialog->setFileMode(options & QFileDialog::ShowDirsOnly ? QFileDialog::DirectoryOnly : QFileDialog::Directory);
    pDialog->setOptions(options);
    pDialog->selectFile(initialSelection(dir));
	pDialog->setWindowTitle(QCoreApplication::translate("FramelessFileDialog", "Select Directory", nullptr));
    pDialog->setSupportedSchemes(supportedSchemes);
    if (pDialog->exec() == QDialog::Accepted)
        return pDialog->selectedUrls().value(0);

    return QUrl();
}

QStringList FramelessFileDialog::getOpenFileNames(QWidget *parent, const QString &caption, const QString &dir, const QString &filter, QString *selectedFilter, QFileDialog::Options options)
{
    const QStringList schemes = QStringList(QStringLiteral("file"));
    const QList<QUrl> selectedUrls = getOpenFileUrls(parent, caption, QUrl::fromLocalFile(dir), filter, selectedFilter, options, schemes);
    QStringList fileNames;
    fileNames.reserve(selectedUrls.size());
    for (const QUrl &url : selectedUrls)
        fileNames << url.toLocalFile();

    return fileNames;
}

QList<QUrl> FramelessFileDialog::getOpenFileUrls(QWidget *parent, const QString &caption, const QUrl &dir, const QString &filter, QString *selectedFilter, QFileDialog::Options options, const QStringList &supportedSchemes)
{
//    FileDialogArgs args;
//    args.parent = parent;
//    args.caption = caption;
//    args.directory = workingDirectory(dir);
//    args.selection = initialSelection(dir);
//    args.filter = filter;
//    args.mode = QFileDialog::ExistingFiles;
//    args.options = options;

//    FileDialog dialog(args);
    FramelessFileDialog* pDialog = new FramelessFileDialog(parent, caption, workingDirectory(dir).toLocalFile(), filter);
    pDialog->setFileMode(QFileDialog::ExistingFiles);
    pDialog->setOptions(options);
    pDialog->selectFile(initialSelection(dir));
	pDialog->setWindowTitle(QCoreApplication::translate("FramelessFileDialog", "Open Files", nullptr));
    pDialog->setSupportedSchemes(supportedSchemes);
    //if (selectedFilter && !selectedFilter->isEmpty())
    //    pDialog->selectNameFilter(*selectedFilter);
    if (pDialog->exec() == QDialog::Accepted) {
        if (selectedFilter)
            *selectedFilter = pDialog->selectedNameFilter();
        return pDialog->selectedUrls();
    }
    return QList<QUrl>();
}

//FileDialog::FileDialog(const FileDialogArgs &args)
//{
//    QHBoxLayout* playout = new QHBoxLayout;
//    playout->setMargin(0);
//    playout->setSpacing(0);
//    m_pFileDialog = new QFileDialog(args.parent, args.caption, args.directory.toLocalFile(), args.filter);

//    m_pFileDialog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    m_pFileDialog->setWindowFlags((m_pFileDialog->windowFlags() | Qt::Widget) & (~Qt::Dialog));
//    playout->addWidget(m_pFileDialog);

//    setLayout(playout);
////    connect(m_pFileDialog, &QFileDialog::rejected, this, &FileDialog::rejected);
////    connect(m_pFileDialog, &QFileDialog::accepted, this, &FileDialog::accepted);

//    setFileMode(args.mode);
//    setOptions(args.options);
//    selectFile(args.selection);
//    resize(800, 600);
//}


/*
    Get the initial directory URL

    \sa initialSelection()
 */
QUrl FramelessFileDialog::workingDirectory(const QUrl &url)
{
    if (!url.isEmpty()) {
        QUrl directory = _qt_get_directory(url);
        if (!directory.isEmpty())
            return directory;
    }
    QUrl directory = _qt_get_directory(*lastVisitedDir());
    if (!directory.isEmpty())
        return directory;
    return QUrl::fromLocalFile(QDir::currentPath());
}

/*
    Get the initial selection given a path.  The initial directory
    can contain both the initial directory and initial selection
    /home/user/foo.txt

    \sa workingDirectory()
 */
QString FramelessFileDialog::initialSelection(const QUrl &url)
{
    if (url.isEmpty())
        return QString();
    if (url.isLocalFile()) {
        QFileInfo info(url.toLocalFile());
        if (!info.isDir())
            return info.fileName();
        else
            return QString();
    }
    // With remote URLs we can only assume.
    return url.fileName();
}

void FramelessFileDialog::slot_rejected()
{
    qDebug()<<"FileDialog::slot_rejected";
    reject();
}

void FramelessFileDialog::slot_accepted()
{
    qDebug()<<"FileDialog::slot_accepted";
    accept();
}

void FramelessFileDialog::connectSignals()
{
    if(nullptr == m_pFileDialog)
        return ;

    connect(m_pFileDialog, &QFileDialog::currentChanged,     this, &FramelessFileDialog::currentChanged);
    connect(m_pFileDialog, &QFileDialog::currentUrlChanged,  this, &FramelessFileDialog::currentUrlChanged);
    connect(m_pFileDialog, &QFileDialog::directoryEntered,   this, &FramelessFileDialog::directoryEntered);
    connect(m_pFileDialog, &QFileDialog::directoryUrlEntered,this, &FramelessFileDialog::directoryUrlEntered);
    connect(m_pFileDialog, &QFileDialog::fileSelected,       this, &FramelessFileDialog::fileSelected);
    connect(m_pFileDialog, &QFileDialog::filesSelected,      this, &FramelessFileDialog::filesSelected);
    connect(m_pFileDialog, &QFileDialog::filterSelected,     this, &FramelessFileDialog::filterSelected);
    connect(m_pFileDialog, &QFileDialog::urlSelected,        this, &FramelessFileDialog::urlSelected);
    connect(m_pFileDialog, &QFileDialog::urlsSelected,       this, &FramelessFileDialog::urlsSelected);
}

