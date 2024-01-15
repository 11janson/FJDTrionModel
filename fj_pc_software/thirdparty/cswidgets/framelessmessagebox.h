#pragma once
#include "cswidgets_global.h"
#include "framelessdialog.h"
#include <QDialog>
#include <QMessageBox>
#include <QString>
#include "QDialogButtonBox"
QT_BEGIN_NAMESPACE
class QLabel;
class QMessageBoxPrivate;
class QAbstractButton;
class QCheckBox;
QT_END_NAMESPACE
namespace CS {
    namespace Widgets {
        class FramelessMessageBoxPrivate;
        class CSWIDGETS_EXPORT FramelessMessageBox : public FramelessDialog
        {
            Q_OBJECT
                Q_PROPERTY(QString text READ text WRITE setText)
                Q_PROPERTY(QMessageBox::Icon icon READ icon WRITE setIcon)
                Q_PROPERTY(QPixmap iconPixmap READ iconPixmap WRITE setIconPixmap)
                Q_PROPERTY(Qt::TextFormat textFormat READ textFormat WRITE setTextFormat)
                Q_PROPERTY(QMessageBox::StandardButtons standardButtons READ standardButtons WRITE setStandardButtons)
#ifndef QT_NO_TEXTEDIT
                Q_PROPERTY(QString detailedText READ detailedText WRITE setDetailedText)
#endif
                Q_PROPERTY(QString informativeText READ informativeText WRITE setInformativeText)
                Q_PROPERTY(Qt::TextInteractionFlags textInteractionFlags READ textInteractionFlags WRITE setTextInteractionFlags)

        public:
            explicit FramelessMessageBox(QWidget *parent = Q_NULLPTR);
            FramelessMessageBox(QMessageBox::Icon icon,
                const QString &title,
                const QString &text,
                QMessageBox::StandardButtons buttons = QMessageBox::NoButton,
                QWidget *parent = Q_NULLPTR,
                Qt::WindowFlags flags = Qt::Dialog | Qt::MSWindowsFixedSizeDialogHint);
            ~FramelessMessageBox();
            void setButtonIsVisiable(QDialogButtonBox::StandardButton buttontype, bool isshow);
			void changeButtonText(QDialogButtonBox::StandardButton buttontype,QString text);
            void addButton(QAbstractButton *button, QMessageBox::ButtonRole role);
            QPushButton *addButton(const QString &text, QMessageBox::ButtonRole role);
            QPushButton *addButton(QMessageBox::StandardButton button);
            void removeButton(QAbstractButton *button);
            using QDialog::open;
            void open(QObject *receiver, const char *member);

            QList<QAbstractButton *> buttons() const;
            QMessageBox::ButtonRole buttonRole(QAbstractButton *button) const;

            void setStandardButtons(QMessageBox::StandardButtons buttons);
            QMessageBox::StandardButtons standardButtons() const;
            QMessageBox::StandardButton standardButton(QAbstractButton *button) const;
            QAbstractButton *button(QMessageBox::StandardButton which) const;

            QPushButton *defaultButton() const;
            void setDefaultButton(QPushButton *button);
            void setDefaultButton(QMessageBox::StandardButton button);

            QAbstractButton *escapeButton() const;
            void setEscapeButton(QAbstractButton *button);
            void setEscapeButton(QMessageBox::StandardButton button);

            QAbstractButton *clickedButton() const;

            QString text() const;
            void setText(const QString &text);

            QMessageBox::Icon icon() const;
            void setIcon(QMessageBox::Icon);

            QPixmap iconPixmap() const;
            void setIconPixmap(const QPixmap &pixmap);

            Qt::TextFormat textFormat() const;
            void setTextFormat(Qt::TextFormat format);

            void setTextInteractionFlags(Qt::TextInteractionFlags flags);
            Qt::TextInteractionFlags textInteractionFlags() const;

            void setCheckBox(QCheckBox *cb);
            QCheckBox* checkBox() const;

			void setDefaultOKButton();

            static QMessageBox::StandardButton information(QWidget *parent, const QString &title,
                const QString &text, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton);
            static QMessageBox::StandardButton information(QWidget *parent, const QString &title,
                const QString &text, char *propertyName, bool propertyVal, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton, bool bShowCloseButton = true);

            static QMessageBox::StandardButton question(QWidget *parent, const QString &title,
                const QString &text, QMessageBox::StandardButtons buttons = QMessageBox::StandardButtons(QMessageBox::Yes | QMessageBox::No),
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton);

            static QMessageBox::StandardButton question(QWidget *parent, const QString &title,
                const QString &text, char *propertyName, bool propertyVal, QMessageBox::StandardButtons buttons = QMessageBox::StandardButtons(QMessageBox::Yes | QMessageBox::No),
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton, bool bShowCloseButton = true);

            static QMessageBox::StandardButton warning(QWidget *parent, const QString &title,
                const QString &text, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton);
            static QMessageBox::StandardButton warning(QWidget *parent, const QString &title, const QString& text,
                char *propertyName, bool propertyVal, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton, bool bShowCloseButton = true);

            static QMessageBox::StandardButton critical(QWidget *parent, const QString &title,
                const QString &text, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton);
            static QMessageBox::StandardButton critical(QWidget *parent, const QString &title,
                const QString &text, char *propertyName, bool propertyVal, QMessageBox::StandardButtons buttons = QMessageBox::Ok,
                QMessageBox::StandardButton defaultButton = QMessageBox::NoButton, bool bShowCloseButton = true);

            static void about(QWidget *parent, const QString &title, const QString &text);
            static void aboutQt(QWidget *parent, const QString &title = QString());

            QString informativeText() const;
            void setInformativeText(const QString &text);

#ifndef QT_NO_TEXTEDIT
            QString detailedText() const;
            void setDetailedText(const QString &text);
#endif

            void setWindowTitle(const QString &title);
            void setWindowModality(Qt::WindowModality windowModality);
            /**
             * @brief 实时翻译时，刷新UI文字显示
             */
            void retranslateButtonUi(QMessageBox::StandardButton button);
        Q_SIGNALS:
            void buttonClicked(QAbstractButton *button);


        protected:
            bool event(QEvent *e) Q_DECL_OVERRIDE;
            void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;
            void showEvent(QShowEvent *event) Q_DECL_OVERRIDE;
            void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;
            void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
            void changeEvent(QEvent *event) Q_DECL_OVERRIDE;
            private slots:
            void slot_buttonClicked(QAbstractButton *);
            void slot_clicked(QMessageBox::StandardButton button, QMessageBox::ButtonRole role);
        private:
            Q_DISABLE_COPY(FramelessMessageBox)
                Q_DECLARE_PRIVATE(FramelessMessageBox)

                FramelessMessageBoxPrivate *d;
        };
    }// namespace Widgets
}// namespace NQ
