
#ifndef DEPTHVISTA_EXPORT_H
#define DEPTHVISTA_EXPORT_H
#ifdef _WIN32
  #ifdef DEPTHVISTA_BUILT_AS_STATIC
  #  define DEPTHVISTA_EXPORT
  #  define DEPTHVISTA_NO_EXPORT
  #else
  #  ifndef DEPTHVISTA_EXPORT
          /* We are building this library */
  #      define DEPTHVISTA_EXPORT __declspec(dllexport)
  #  endif

  #  ifndef DEPTHVISTA_NO_EXPORT
  #    define DEPTHVISTA_NO_EXPORT
  #  endif
  #endif

  #ifndef DEPTHVISTA_DEPRECATED
  #  define DEPTHVISTA_DEPRECATED __declspec(deprecated)
  #endif

  #ifndef DEPTHVISTA_DEPRECATED_EXPORT
  #  define DEPTHVISTA_DEPRECATED_EXPORT DEPTHVISTA_EXPORT DEPTHVISTA_DEPRECATED
  #endif

  #ifndef DEPTHVISTA_DEPRECATED_NO_EXPORT
  #  define DEPTHVISTA_DEPRECATED_NO_EXPORT DEPTHVISTA_NO_EXPORT DEPTHVISTA_DEPRECATED
  #endif

  #if 0 /* DEFINE_NO_DEPRECATED */
  #  ifndef DEPTHVISTA_NO_DEPRECATED
  #    define DEPTHVISTA_NO_DEPRECATED
  #  endif
  #endif
#elif __linux__
  #define DEPTHVISTA_EXPORT extern "C"
#endif
#endif /* DEPTHVISTA_EXPORT_H */
