#pragma once
/*
   The operating system, must be one of: (Q_OS_x)

     DARWIN   - Any Darwin system (macOS, iOS, watchOS, tvOS)
     MACOS    - macOS
     IOS      - iOS
     WATCHOS  - watchOS
     TVOS     - tvOS
     MSDOS    - MS-DOS and Windows
     OS2      - OS/2
     OS2EMX   - XFree86 on OS/2 (not PM)
     WIN32    - Win32 (Windows 2000/XP/Vista/7 and Windows Server 2003/2008)
     WINRT    - WinRT (Windows 8 Runtime)
     CYGWIN   - Cygwin
     SOLARIS  - Sun Solaris
     HPUX     - HP-UX
     ULTRIX   - DEC Ultrix
     LINUX    - Linux [has variants]
     FREEBSD  - FreeBSD [has variants]
     NETBSD   - NetBSD
     OPENBSD  - OpenBSD
     BSDI     - BSD/OS
     INTERIX  - Interix
     IRIX     - SGI Irix
     OSF      - HP Tru64 UNIX
     SCO      - SCO OpenServer 5
     UNIXWARE - UnixWare 7, Open UNIX 8
     AIX      - AIX
     HURD     - GNU Hurd
     DGUX     - DG/UX
     RELIANT  - Reliant UNIX
     DYNIX    - DYNIX/ptx
     QNX      - QNX [has variants]
     QNX6     - QNX RTP 6.1
     LYNX     - LynxOS
     BSD4     - Any BSD 4.4 system
     UNIX     - Any UNIX BSD/SYSV system
     ANDROID  - Android platform
     HAIKU    - Haiku

   The following operating systems have variants:
     LINUX    - both Q_OS_LINUX and Q_OS_ANDROID are defined when building for Android
              - only Q_OS_LINUX is defined if building for other Linux systems
     FREEBSD  - Q_OS_FREEBSD is defined only when building for FreeBSD with a BSD userland
              - Q_OS_FREEBSD_KERNEL is always defined on FreeBSD, even if the userland is from GNU
*/

#if defined(__APPLE__) && (defined(__GNUC__) || defined(__xlC__) || defined(__xlc__))
#  include <TargetConditionals.h>
#  if defined(TARGET_OS_MAC) && TARGET_OS_MAC
#    define NC_OS_DARWIN
#    define NC_OS_BSD4
#    ifdef __LP64__
#      define NC_OS_DARWIN64
#    else
#      define NC_OS_DARWIN32
#    endif
#    if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
#      define NC_PLATFORM_UIKIT
#      if defined(TARGET_OS_WATCH) && TARGET_OS_WATCH
#        define NC_OS_WATCHOS
#      elif defined(TARGET_OS_TV) && TARGET_OS_TV
#        define NC_OS_TVOS
#      else
#        // TARGET_OS_IOS is only available in newer SDKs,
#        // so assume any other iOS-based platform is iOS for now
#        define NC_OS_IOS
#      endif
#    else
#      // TARGET_OS_OSX is only available in newer SDKs,
#      // so assume any non iOS-based platform is macOS for now
#      define NC_OS_MACOS
#    endif
#  else
#    error "Nova has not been ported to this Apple platform - see http://www.qt.io/developers"
#  endif
#elif defined(__CYGWIN__)
#  define NC_OS_CYGWIN
#elif !defined(SAG_COM) && (!defined(WINAPI_FAMILY) || WINAPI_FAMILY==WINAPI_FAMILY_DESKTOP_APP) && (defined(WIN64) || defined(_WIN64) || defined(__WIN64__))
#  define NC_OS_WIN32
#  define NC_OS_WIN64
#elif !defined(SAG_COM) && (defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__))
#  if defined(WINAPI_FAMILY)
#    ifndef WINAPI_FAMILY_PC_APP
#      define WINAPI_FAMILY_PC_APP WINAPI_FAMILY_APP
#    endif
#    if defined(WINAPI_FAMILY_PHONE_APP) && WINAPI_FAMILY==WINAPI_FAMILY_PHONE_APP
#      define NC_OS_WINRT
#    elif WINAPI_FAMILY==WINAPI_FAMILY_PC_APP
#      define NC_OS_WINRT
#    else
#      define NC_OS_WIN32
#    endif
#  else
#    define NC_OS_WIN32
#  endif
#elif defined(__linux__) || defined(__linux)
#  define NC_OS_LINUX
#else
#  error "Nova has not been ported to this OS - see http://www.qt-project.org/"
#endif

#if defined(NC_OS_WIN32) || defined(NC_OS_WIN64) || defined(NC_OS_WINRT)
#  define NC_OS_WIN
#endif

#if defined(NC_OS_WIN)
#  undef NC_OS_UNIX
#elif !defined(NC_OS_UNIX)
#  define NC_OS_UNIX
#endif

// Compatibility synonyms
#ifdef NC_OS_DARWIN
#define NC_OS_MAC
#endif
#ifdef NC_OS_DARWIN32
#define NC_OS_MAC32
#endif
#ifdef NC_OS_DARWIN64
#define NC_OS_MAC64
#endif
#ifdef NC_OS_MACOS
#define NC_OS_MACX
#define NC_OS_OSX
#endif

#ifdef NC_OS_WIN
#  define NC_DECL_EXPORT     __declspec(dllexport)
#  define NC_DECL_IMPORT     __declspec(dllimport)
#else
#  define NC_DECL_EXPORT     __attribute__((visibility("default")))
#  define NC_DECL_IMPORT     __attribute__((visibility("default")))
#  define NC_DECL_HIDDEN     __attribute__((visibility("hidden")))
#endif


#ifndef NC_DECL_EXPORT
#  define NC_DECL_EXPORT
#endif
#ifndef NC_DECL_IMPORT
#  define NC_DECL_IMPORT
#endif
#ifndef NC_DECL_HIDDEN
#  define NC_DECL_HIDDEN
#endif
