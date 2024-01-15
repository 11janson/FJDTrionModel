#ifndef FJFRAMELESSPROCESSDIALOG_H
#define FJFRAMELESSPROCESSDIALOG_H
#include <qtwidgetsglobal.h>
#include <QDialog>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QApplication>
#include <QStyle>
#include <QTimer>
#include <QSize>
#include <QResizeEvent>
#include <QCloseEvent>
#include <QShowEvent>
#include <QEvent>
#include "cswidgets_global.h"
#include "qevent.h"

QT_BEGIN_NAMESPACE

class FJframelessprocessdialogPrivate;

class CSWIDGETS_EXPORT FJframelessprocessdialog : public QDialog
{
	Q_OBJECT
		Q_DECLARE_PRIVATE(FJframelessprocessdialog)
		Q_PROPERTY(bool wasCanceled READ wasCanceled)
		Q_PROPERTY(int minimum READ minimum WRITE setMinimum)
		Q_PROPERTY(int maximum READ maximum WRITE setMaximum)
		Q_PROPERTY(int value READ value WRITE setValue)
		Q_PROPERTY(bool autoReset READ autoReset WRITE setAutoReset)
		Q_PROPERTY(bool autoClose READ autoClose WRITE setAutoClose)
		Q_PROPERTY(int minimumDuration READ minimumDuration WRITE setMinimumDuration)
		Q_PROPERTY(QString labelText READ labelText WRITE setLabelText)

public:
	explicit FJframelessprocessdialog(QWidget *parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());
	FJframelessprocessdialog(const QString &labelText, const QString &cancelButtonText,
		int minimum, int maximum, QWidget *parent = nullptr,
		Qt::WindowFlags flags = Qt::WindowFlags());
	~FJframelessprocessdialog();

	void setLabel(QLabel *label);
	void setCancelButton(QPushButton *button);
	void setBar(QProgressBar *bar);
	void setWindowTitle(const QString &text);
	bool wasCanceled() const;

	int minimum() const;
	int maximum() const;

	int value() const;

	virtual QSize sizeHint() const override;

	QString labelText() const;
	int minimumDuration() const;

	void setAutoReset(bool reset);
	bool autoReset() const;
	void setAutoClose(bool close);
	bool autoClose() const;

	using QDialog::open;
	void open(QObject *receiver, const char *member);

public Q_SLOTS:
	void cancel();
	void reset();
	void setMaximum(int maximum);
	void setMinimum(int minimum);
	void setRange(int minimum, int maximum);
	void setValue(int progress);
	void setLabelText(const QString &text);
	void setCancelButtonText(const QString &text);
	void setMinimumDuration(int ms);

Q_SIGNALS:
	void canceled();

protected:
	virtual void resizeEvent(QResizeEvent *event) override;
	virtual void closeEvent(QCloseEvent *event) override;
	virtual void changeEvent(QEvent *event) override;
	virtual void showEvent(QShowEvent *event) override;
	virtual void paintEvent(QPaintEvent *event);
	virtual void keyPressEvent(QKeyEvent *event);

protected Q_SLOTS:
	void forceShow();

private:
	Q_DISABLE_COPY(FJframelessprocessdialog)
	Q_PRIVATE_SLOT(d_func(), void _q_disconnectOnClose())
};

QT_END_NAMESPACE

#endif // FILEDIALOG_H
