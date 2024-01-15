#ifndef QCANUPOGLOBAL_H
#define QCANUPOGLOBAL_H

#if defined(QCANUPO_PLUGIN_EXPORTS)
#  define QCANUPO_EXPORT __declspec(dllexport)
#else
#  define QCANUPO_EXPORT __declspec(dllimport)
#endif

#endif // QCANUPOGLOBAL_H