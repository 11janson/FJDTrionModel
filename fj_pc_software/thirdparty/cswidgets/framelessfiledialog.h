#ifndef FILEDIALOG_H
#define FILEDIALOG_H
#include "cswidgets_global.h"
#include <QDialog>
#include <QFileDialog>
#include "framelessdialog.h"

namespace CS {
namespace Widgets {

class CSWIDGETS_EXPORT FramelessFileDialog : public FramelessDialog
{
    Q_OBJECT
public:
//    explicit FileDialog(QWidget *parent = nullptr);
    FramelessFileDialog(QWidget *parent, Qt::WindowFlags f);
    explicit FramelessFileDialog(QWidget *parent = Q_NULLPTR,
                         const QString &caption = QString(),
                         const QString &directory = QString(),
                         const QString &filter = QString());
signals:

public:

    void setDirectory(const QString &directory);
    inline void setDirectory(const QDir &directory);
    QDir directory() const;

    void setDirectoryUrl(const QUrl &directory);
    QUrl directoryUrl() const;

    void selectFile(const QString &filename);
    QStringList selectedFiles() const;

    void selectUrl(const QUrl &url);
    QList<QUrl> selectedUrls() const;

    void setNameFilterDetailsVisible(bool enabled);
    bool isNameFilterDetailsVisible() const;

    void setNameFilter(const QString &filter);
    void setNameFilters(const QStringList &filters);
    QStringList nameFilters() const;
    void selectNameFilter(const QString &filter);
//    QString selectedMimeTypeFilter() const;
    QString selectedNameFilter() const;

#ifndef QT_NO_MIMETYPE
    void setMimeTypeFilters(const QStringList &filters);
    QStringList mimeTypeFilters() const;
    void selectMimeTypeFilter(const QString &filter);
#endif

    QDir::Filters filter() const;
    void setFilter(QDir::Filters filters);

    void setViewMode(QFileDialog::ViewMode mode);
    QFileDialog::ViewMode viewMode() const;

    void setFileMode(QFileDialog::FileMode mode);
    QFileDialog::FileMode fileMode() const;

    void setAcceptMode(QFileDialog::AcceptMode mode);
    QFileDialog::AcceptMode acceptMode() const;

    void setReadOnly(bool enabled);
    bool isReadOnly() const;

    void setResolveSymlinks(bool enabled);
    bool resolveSymlinks() const;

    void setSidebarUrls(const QList<QUrl> &urls);
    QList<QUrl> sidebarUrls() const;

    QByteArray saveState() const;
    bool restoreState(const QByteArray &state);

    void setConfirmOverwrite(bool enabled);
    bool confirmOverwrite() const;

    void setDefaultSuffix(const QString &suffix);
    QString defaultSuffix() const;

    void setHistory(const QStringList &paths);
    QStringList history() const;

    void setItemDelegate(QAbstractItemDelegate *delegate);
    QAbstractItemDelegate *itemDelegate() const;

    void setIconProvider(QFileIconProvider *provider);
    QFileIconProvider *iconProvider() const;

    void setLabelText(QFileDialog::DialogLabel label, const QString &text);
    QString labelText(QFileDialog::DialogLabel label) const;

    void setSupportedSchemes(const QStringList &schemes);
    QStringList supportedSchemes() const;

#ifndef QT_NO_PROXYMODEL
    void setProxyModel(QAbstractProxyModel *model);
    QAbstractProxyModel *proxyModel() const;
#endif

    void setOption(QFileDialog::Option option, bool on = true);
    bool testOption(QFileDialog::Option option) const;
    void setOptions(QFileDialog::Options options);
    QFileDialog::Options options() const;

//    using QDialog::open;
//    void open(QObject *receiver, const char *member);
//    void setVisible(bool visible) Q_DECL_OVERRIDE;

public:

    static QString getOpenFileName(QWidget *parent = Q_NULLPTR,
                                   const QString &caption = QString(),
                                   const QString &dir = QString(),
                                   const QString &filter = QString(),
                                   QString *selectedFilter = Q_NULLPTR,
                                   QFileDialog::Options options = QFileDialog::Options());

    static QUrl getOpenFileUrl(QWidget *parent = Q_NULLPTR,
                               const QString &caption = QString(),
                               const QUrl &dir = QUrl(),
                               const QString &filter = QString(),
                               QString *selectedFilter = Q_NULLPTR,
                               QFileDialog::Options options = QFileDialog::Options(),
                               const QStringList &supportedSchemes = QStringList());

    static QString getSaveFileName(QWidget *parent = Q_NULLPTR,
                                   const QString &caption = QString(),
                                   const QString &dir = QString(),
                                   const QString &filter = QString(),
                                   QString *selectedFilter = Q_NULLPTR,
                                   QFileDialog::Options options = QFileDialog::Options());

    static QUrl getSaveFileUrl(QWidget *parent = Q_NULLPTR,
                               const QString &caption = QString(),
                               const QUrl &dir = QUrl(),
                               const QString &filter = QString(),
                               QString *selectedFilter = Q_NULLPTR,
                               QFileDialog::Options options = QFileDialog::Options(),
                               const QStringList &supportedSchemes = QStringList());

    static QString getExistingDirectory(QWidget *parent = Q_NULLPTR,
                                        const QString &caption = QString(),
                                        const QString &dir = QString(),
                                        QFileDialog::Options options = QFileDialog::ShowDirsOnly);

    static QUrl getExistingDirectoryUrl(QWidget *parent = Q_NULLPTR,
                                        const QString &caption = QString(),
                                        const QUrl &dir = QUrl(),
                                        QFileDialog::Options options = QFileDialog::ShowDirsOnly,
                                        const QStringList &supportedSchemes = QStringList());

    static QStringList getOpenFileNames(QWidget *parent = Q_NULLPTR,
                                        const QString &caption = QString(),
                                        const QString &dir = QString(),
                                        const QString &filter = QString(),
                                        QString *selectedFilter = Q_NULLPTR,
                                        QFileDialog::Options options = QFileDialog::Options());

    static QList<QUrl> getOpenFileUrls(QWidget *parent = Q_NULLPTR,
                                       const QString &caption = QString(),
                                       const QUrl &dir = QUrl(),
                                       const QString &filter = QString(),
                                       QString *selectedFilter = Q_NULLPTR,
                                       QFileDialog::Options options = QFileDialog::Options(),
                                       const QStringList &supportedSchemes = QStringList());

//protected:
//    FileDialog(const FileDialogArgs &args);
////    virtual int exec();
private:
    static QUrl workingDirectory(const QUrl &url);
    static QString initialSelection(const QUrl &url);

    void slot_rejected();
    void slot_accepted();
    void connectSignals();
signals:
    void currentChanged(const QString &path);
    void currentUrlChanged(const QUrl &url);
    void directoryEntered(const QString &directory);
    void directoryUrlEntered(const QUrl &directory);
    void fileSelected(const QString &file);
    void filesSelected(const QStringList &selected);
    void filterSelected(const QString &filter);
    void urlSelected(const QUrl &url);
    void urlsSelected(const QList<QUrl> &urls);

private:
    QFileDialog* m_pFileDialog = nullptr;
	QLabel * bottomlabel;
};
}
}
#endif // FILEDIALOG_H
