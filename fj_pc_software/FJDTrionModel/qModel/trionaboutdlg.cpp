
#include "trionaboutdlg.h"
#include "ui_trionaboutdlg.h"
#include "titlebar.h"
#include "cloudcompareutils\icore.h"
#include <QPushButton>
#include <QProcess>
#include <QNetworkRequest>
#include <QUrl>
#include <QNetworkAccessManager>
#include <QJsonDocument>
#include "framelessmessagebox.h"
#include <QJsonArray>
#include <QJsonObject>
#include "FJStyleManager.h"
TrionAboutDlg::TrionAboutDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::TrionAboutDlg)
{
	m_ui->setupUi(this->getContentHolder());
	setWindowTitle("about");
	bottomWidget()->setVisible(false);
	//getTitleBar()->setTitleButtonIcon(CS::Widgets::TitleBar::PredefinedButtonType::LogoButton,
	//	CS::Core::ICore::resourceThemeImage("logo.png"));
	getTitleBar()->setVisible(false);
	m_ui->label->setText(tr("About"));
	//m_ui->pushButton->setText(tr("Check updates"));
	m_ui->toolButton_icon->setIcon(CS::Core::ICore::resourceThemeImage("logo.png"));
	QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
	m_ui->toolButton_close->setIcon(closeicon);
	QSettings configSettings(QCoreApplication::applicationDirPath() + "/share/version/versionConfig.ini", QSettings::IniFormat);
	configSettings.beginGroup("TrionModelStart");
	QString mainVersion = configSettings.value("version", "0.9.2").toString();
	QString secondVersion = configSettings.value("packageversion", "").toString();
	QString allmainVersion = mainVersion;
	if (!secondVersion.isEmpty())
	{
		allmainVersion = mainVersion + "+" + secondVersion;
	}
	configSettings.endGroup();
	QSettings Settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	Settings.beginGroup("TrionModelStart");
	QString confmainVersion = Settings.value("version", "").toString();
	QString confsecondVersion = Settings.value("packageversion", "").toString();
	Settings.setValue("version", mainVersion);
	Settings.setValue("packageversion", secondVersion);
	if (confmainVersion!= mainVersion || confsecondVersion != secondVersion)
	{
		Settings.setValue("Versionid", -1);
	}
	Settings.endGroup();
	m_ui->label_versiov->setText(tr("Version:")+ allmainVersion);
	m_ui->toolButton->setIcon(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/aboutlogo.png"));
	connect(m_ui->pushButton, &QPushButton::clicked, this, &TrionAboutDlg::checkVersion);
	connect(m_ui->toolButton_close, &QPushButton::clicked, this, &TrionAboutDlg::close);
}

TrionAboutDlg::~TrionAboutDlg()
{
	delete m_ui;
}

void TrionAboutDlg::checkVersion()
{
	QSettings configSettings(QCoreApplication::applicationDirPath() + "/share/version/versionConfig.ini", QSettings::IniFormat);
	configSettings.beginGroup("TrionModelStart");
	QString mainVersion = configSettings.value("version", "0.9.2").toString();
	//QString secondVersion = configSettings.value("packageversion", "").toString();
	//if (!secondVersion.isEmpty())
	//{
	//	mainVersion = mainVersion + "+" + secondVersion;
	//}
	//QString currentVersion = configSettings.value("version", "0.9.2").toString();
	QString currentVersionQuestUrl = configSettings.value("versionQuestUrl", "http://192.168.32.28:10590/scanner/patch/targetPatches").toString();
	configSettings.endGroup();
	QUrl url(currentVersionQuestUrl);
	QNetworkRequest req;
	QNetworkAccessManager nam;
	if (nam.networkAccessible() == QNetworkAccessManager::NotAccessible) {
		nam.setNetworkAccessible(QNetworkAccessManager::Accessible);
	}
	connect(&nam, &QNetworkAccessManager::finished, this, &TrionAboutDlg::requestRemoteVersionFinished);

	QUrlQuery quurl;
	quurl.addQueryItem("targetVersion", mainVersion);
	url.setQuery(quurl);
	req.setUrl(url);
	QNetworkReply *reply = nam.get(req);
	QEventLoop loop;
	connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
	loop.exec();
}

void TrionAboutDlg::requestRemoteVersionFinished(QNetworkReply *reply) {
	QString rsdata = reply->readAll();
	QJsonParseError jsonParseError;
	QJsonDocument jsonDoc(QJsonDocument::fromJson(rsdata.toUtf8(), &jsonParseError));
	if (QJsonParseError::NoError != jsonParseError.error) {
		CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Network error."));
		return;
	}
	QJsonArray dataObj = (jsonDoc.object().value("data")).toArray();
	if (dataObj.size()==0) {
		CS::Widgets::FramelessMessageBox::information(this, tr("Tip"), tr("Currently is the latest version."));
		return;
	}
	int maxIdIndex = 0;
	int maxid = -99999;
	for (int i = 0;i < dataObj.size();i++)
	{
		int curid = dataObj[i].toObject().value("id").toInt();
		if (curid > maxid)
		{
			maxid = curid;
			maxIdIndex = i;
		}
	}
	QSettings configSettings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	configSettings.beginGroup("TrionModelStart");
	int Versionid = configSettings.value("Versionid", -1).toInt();
	configSettings.endGroup();
	if (maxid <= Versionid)
	{
		CS::Widgets::FramelessMessageBox::information(this, tr("Tip"), tr("Currently is the latest version."));
		return;
	}

	QString patchUrl = dataObj[maxIdIndex].toObject().value("patchUrl").toString();
	QString patchName = dataObj[maxIdIndex].toObject().value("patchName").toString();
	if (patchUrl.isEmpty()) {
		CS::Widgets::FramelessMessageBox::information(this, tr("Tip"), tr("Currently is the latest version!"));
		return;
	}
	CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question,
        tr("Check for Update"),
        tr("Update available:") + patchName,
		QMessageBox::Cancel | QMessageBox::Yes,
		this);
	message_box.changeButtonText(QDialogButtonBox::StandardButton::Yes, tr("Update"));
	if (message_box.exec() != QMessageBox::Yes)
	{
		return;
	}
	QString Programpath = QCoreApplication::applicationDirPath() + QString("/") + QString("TrionModelStart.exe");
	QProcess * process = new QProcess();
	process->setProgram(Programpath);
	if (process->startDetached())
	{
		close();
	}
}


