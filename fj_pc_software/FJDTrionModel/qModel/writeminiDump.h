#pragma once

#include <windows.h>
#include <DbgHelp.h>
#include <winnt.h>
#include <time.h>
#include <locale>
#include <codecvt>

#include <tchar.h>
#pragma comment( lib, "Dbghelp.lib" )
#include "crashreturnfunction.h"
#include<QDebug>
#include"errhandlingapi.h"
#include "ncspdlog/spdlogapplication.h"

void outLogMessageToFile(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    if (type == QtDebugMsg)
    {
        QString message;
        message = qFormatLogMessage(type, context, msg);
        message.append("\r\n");

        QFile file(CS::Core::ICore::getDefaultPath() + "/log/ModelCrashBack.log"); //写文件路径
        if (file.open(QIODevice::WriteOnly | QIODevice::Append))
        {
            file.write(message.toLocal8Bit());
        }

        file.close();
    }
}


LONG __stdcall ApplicationCrashHandler(EXCEPTION_POINTERS* pException)
{
	//[!]关闭日志
	NC::SpdlogApplication::instance()->flush();
	NC::SpdlogApplication::instance()->shutdownLogger();
	spdlog::drop_all();

#if StartCrashApplication   

    // 设置qDebug输出格式
    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss}] %{file} %{line} %{function} %{message}");
    // 安装消息器
    qInstallMessageHandler(outLogMessageToFile);
#endif
    //创建 Dump 文件
    QDateTime CurDTime = QDateTime::currentDateTime();
    std::string timeStr = CurDTime.toString("yyyy_MM_dd_hh_mm_ss").toStdString();
    //dmp文件的命名
    std::string dumpFilePath = CS::Core::ICore::getDefaultPath().toStdString() + "/dump/" + timeStr + ".dmp";
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    std::wstring wide = converter.from_bytes(dumpFilePath);
    EXCEPTION_RECORD *record = pException->ExceptionRecord;
    QString errCode(QString::number(record->ExceptionCode));
    QString errAddr(QString::number((uint)record->ExceptionAddress));
    QString errFlag(QString::number(record->ExceptionFlags));
    QString errPara(QString::number(record->NumberParameters));
    HANDLE DumpHandle = CreateFile(wide.c_str(),
        GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (DumpHandle != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
        dumpInfo.ExceptionPointers = pException;
        dumpInfo.ThreadId = GetCurrentThreadId();
        dumpInfo.ClientPointers = TRUE;
        //将dump信息写入dmp文件
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), DumpHandle, MiniDumpWithDataSegs, &dumpInfo, NULL, NULL);
        CloseHandle(DumpHandle);
    }

   
    qDebug() << "EXCEPTION_EXECUTE_HANDLER:" << EXCEPTION_EXECUTE_HANDLER;
#if StartCrashApplication   
    //创建消息提示
	CS::Widgets::FramelessMessageBox::warning(nullptr, "Error", " Sorry , FJD TrionModel Crash!");
    crashreturnfunction postObject;
    QProcess process;
    QStringList param;
    QString exePath = postObject.getCrashExePath();
    param = postObject.getCrashExePram();
    qDebug() << "param" << param.size();
    for (int i = 0; i < param.size(); i++)
    {
        qDebug() << "param:" << i << param.at(i);
    }
    process.startDetached(exePath, param);
    process.waitForStarted();
#endif
    return EXCEPTION_EXECUTE_HANDLER;

}

static BOOL PreventFuncall(void *oldfun, void *newfun)
{
    void* pOrgEntry = oldfun;
    if (pOrgEntry == NULL)
        return FALSE;
    DWORD dwOldProtect = 0;
    SIZE_T jmpSize = 5;
#ifdef _M_X64
    jmpSize = 13;
#endif
    BOOL bProt = VirtualProtect(pOrgEntry, jmpSize, PAGE_EXECUTE_READWRITE, &dwOldProtect);
    BYTE newJump[20];

    void* pNewFunc = newfun;
#ifdef _M_IX86
    DWORD dwOrgEntryAddr = (DWORD)pOrgEntry;
    dwOrgEntryAddr += jmpSize;
    DWORD dwNewEntryAddr = (DWORD)pNewFunc;
    DWORD dwRelativeAddr = dwNewEntryAddr - dwOrgEntryAddr;
    newJump[0] = 0xE9;
    memcpy(&newJump[1], &dwRelativeAddr, sizeof(pNewFunc));
#elif _M_X64
    newJump[0] = 0x49;
    newJump[1] = 0xBB;

    memcpy(&newJump[2], &pNewFunc, sizeof(pNewFunc));

    newJump[10] = 0x41;
    newJump[11] = 0xFF;
    newJump[12] = 0xE3;
#endif

    SIZE_T bytesWritten;

    BOOL bRet = WriteProcessMemory(GetCurrentProcess(), pOrgEntry, newJump, jmpSize, &bytesWritten);

    if (bProt != FALSE) {
        DWORD dwBuf;
        VirtualProtect(pOrgEntry, jmpSize, dwOldProtect, &dwBuf);
    }
    return bRet;
}
void DisableSetUnhandledExceptionFilter()
{

    HMODULE h_hand = LoadLibrary(TEXT("kernel32.dll"));

    void* addr = (void*)GetProcAddress(h_hand, "SetUnhandledExceptionFilter");

    void *newfun = nullptr;
    //PreventFuncall(addr, newfun);


    if (addr)
    {
        unsigned char code[16];

        int size = 0;

        code[size++] = 0x33;

        code[size++] = 0xC0;

        code[size++] = 0xC2;

        code[size++] = 0x00;

        code[size++] = 0x00;

        DWORD dwOldFlag, dwTempFlag;
         //[!]内存映射
        VirtualProtect(addr, size, PAGE_READWRITE, &dwOldFlag);
        qDebug("WriteProcessMemory start!");
        WriteProcessMemory(GetCurrentProcess(), addr, code, size, NULL);
        qDebug("WriteProcessMemory end!");
        VirtualProtect(addr, size, dwOldFlag, &dwTempFlag);
    }
}

void RunCrashHandlerLocal()
{

    SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);
   
#if StartCrashApplication   

	DisableSetUnhandledExceptionFilter();
#endif

	QString paths = CS::Core::ICore::getDefaultPath() + "/dump";
	QDir dir(paths);
	if (!dir.exists())
	{
		dir.mkdir(paths);
	}
}
