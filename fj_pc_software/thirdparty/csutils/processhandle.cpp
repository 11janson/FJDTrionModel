/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of Qt Creator.
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 as published by the Free Software
** Foundation with exceptions as appearing in the file LICENSE.GPL3-EXCEPT
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
****************************************************************************/

#include "processhandle.h"
#include <QString>
namespace Utils {

/*!
    \class Utils::ProcessHandle
    \brief The ProcessHandle class is a helper class to describe a process.

    Encapsulates parameters of a running process, local (PID) or remote (to be
    done, address, port, and so on).
*/

// That's the same as in QProcess, i.e. Qt doesn't care for process #0.
const qint64 InvalidPid = 0;

ProcessHandle::ProcessHandle()
    : m_pid(InvalidPid)
{
}

ProcessHandle::ProcessHandle(qint64 pid)
    : m_pid(pid)
{
}

bool ProcessHandle::isValid() const
{
    return m_pid != InvalidPid;
}

void ProcessHandle::setPid(qint64 pid)
{
    m_pid = pid;
}

qint64 ProcessHandle::pid() const
{
    return m_pid;
}

bool ProcessHandle::equals(const ProcessHandle &rhs) const
{
    return m_pid == rhs.m_pid;
}

#ifndef Q_OS_OSX
bool ProcessHandle::activate()
{
    return false;
}
#endif


bool ProcessHandle::isExist(const QString& strProceesName)
{
#ifdef Q_OS_WIN
	return isExistWin(strProceesName);
#elif defined Q_OS_UNIX
	return isExistLinux(strProceesName);
#elif defined Q_OS_MAC
	return isExistMac(strProceesName);
#else
	return false;
#endif //  Q_OS_WIN
}

#ifdef Q_OS_WIN
#include "windows.h"
#include "tlhelp32.h"

bool ProcessHandle::isExistWin(const QString& strProceesName)
{
	PROCESSENTRY32 pe;

	HANDLE hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);

	pe.dwSize = sizeof(PROCESSENTRY32);

	if (!Process32First(hSnapshot, &pe))

		return 0;

	while (1)
	{
		pe.dwSize = sizeof(PROCESSENTRY32);

		if (Process32Next(hSnapshot, &pe) == FALSE)
			break;

		if (wcscmp(pe.szExeFile, strProceesName.toStdWString().c_str()) == 0)
		{
			CloseHandle(hSnapshot);
			return true;
		}
	}

	CloseHandle(hSnapshot);

	return false;
}
#endif

#ifdef Q_OS_UNIX
bool ProcessHandle::isExistLinux(const QString& strProceesName)
{
	return false;
}
#endif

#ifdef Q_OS_MAC
bool ProcessHandle::isExistMac(const QString& strProceesName)
{
	return false;
}
#endif

} // Utils
