#pragma once

#if defined(__APPLE__)
#include <mach-o/dyld.h> // _NSGetExecutablePath
#elif defined(__linux__)
#include <unistd.h> // readlink
#endif


namespace NC
{
    class BinaryLocation
    {
    public:
#define SET_BINARY_PATH_ERROR_MESSAGE(path, msg) \
    str_len = (int) strlen(msg);                 \
    memcpy(path, msg, (size_t)str_len);          \
    path[str_len] = char(0)

#ifdef _WIN32
        static bool Get(char *path, const unsigned size)
        {
            // TODO: make AssertMsg available under PlatformAgnostic
            //AssertMsg(path != nullptr, "Path can not be nullptr");
            //AssertMsg(size < INT_MAX, "Isn't it too big for a path buffer?");
            LPWSTR wpath = (WCHAR*)malloc(sizeof(WCHAR) * size);
            int str_len;
            if (!wpath)
            {
                SET_BINARY_PATH_ERROR_MESSAGE(path, "GetBinaryLocation: GetModuleFileName has failed. OutOfMemory!");
                return false;
            }
            str_len = GetModuleFileNameW(NULL, wpath, size - 1);
            if (str_len <= 0)
            {
                SET_BINARY_PATH_ERROR_MESSAGE(path, "GetBinaryLocation: GetModuleFileName has failed.");
                free(wpath);
                return false;
            }

            str_len = WideCharToMultiByte(CP_UTF8, 0, wpath, str_len, path, size, NULL, NULL);
            free(wpath);

            if (str_len <= 0)
            {
                SET_BINARY_PATH_ERROR_MESSAGE(path, "GetBinaryLocation: GetModuleFileName (WideCharToMultiByte) has failed.");
                return false;
            }

            if ((unsigned)str_len > size - 1)
            {
                str_len = (int)size - 1;
            }
            path[str_len] = char(0);
            return true;
        }

        // Overloaded GetBinaryLocation: receive the location of current binary in char16.
        // size: the second parameter is the size of the path buffer in count of wide characters.
        static bool GetBinaryLocation(char16 *path, const unsigned size)
        {
            // TODO: make AssertMsg available under PlatformAgnostic
            //AssertMsg(path != nullptr, "Path can not be nullptr");
            //AssertMsg(size < INT_MAX, "Isn't it too big for a path buffer?");
            int str_len = GetModuleFileNameW(NULL, path, size);
            if (str_len <= 0)
            {
                wcscpy_s(path, size, _u("GetBinaryLocation: GetModuleFileName has failed."));
                return false;
            }
            return true;
        }
#else
        static bool Get(char *path, const unsigned size)
        {
            // TODO: make AssertMsg available under PlatformAgnostic
            //AssertMsg(path != nullptr, "Path can not be nullptr");
            //AssertMsg(size < INT_MAX, "Isn't it too big for a path buffer?");
    #ifdef __APPLE__
            uint32_t path_size = (uint32_t)size;
            char *tmp = nullptr;
            int str_len;
            if (_NSGetExecutablePath(path, &path_size))
            {
                SET_BINARY_PATH_ERROR_MESSAGE(path, "GetBinaryLocation: _NSGetExecutablePath has failed.");
                return false;
            }

            tmp = (char*)malloc(size);
            char *result = realpath(path, tmp);
            str_len = strlen(result);
            memcpy(path, result, str_len);
            free(tmp);
            path[str_len] = char(0);
            return true;
    #elif defined(__linux__)
            int str_len = readlink("/proc/self/exe", path, size - 1);
            if (str_len <= 0)
            {
                SET_BINARY_PATH_ERROR_MESSAGE(path, "GetBinaryLocation: /proc/self/exe has failed.");
                return false;
            }
            path[str_len] = char(0);
            return true;
    #else
            #warning "Implement GetBinaryLocation for this platform"
    #endif
        }

//        static bool Get(wchar_t *path, const unsigned size)
//        {
//            int tmp_size = size * 3;
//            char *tmp = (char*)malloc(tmp_size);
//            if (!Get(tmp, tmp_size))
//            {
//                free(tmp);
//                return false;
//            }
//            if (utf8::DecodeUnitsIntoAndNullTerminate(path, (LPCUTF8&)tmp, (LPCUTF8)tmp + strlen(tmp)) <= 0)
//            {
//                free(tmp);
//                wcscpy_s(path, size, _u("GetBinaryLocation: DecodeUnitsIntoAndNullTerminate has failed."));
//                return false;
//            }
//            free(tmp);
//            return true;
//        }
#endif
    };
} // namespace NC
