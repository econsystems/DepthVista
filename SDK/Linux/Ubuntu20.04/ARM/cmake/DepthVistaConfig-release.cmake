#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "DepthVistaSDK" for configuration "Release"
set_property(TARGET DepthVistaSDK APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(DepthVistaSDK PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libDepthVistaSDK.so.1.0.10"
  IMPORTED_SONAME_RELEASE "libDepthVistaSDK.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS DepthVistaSDK )
list(APPEND _IMPORT_CHECK_FILES_FOR_DepthVistaSDK "${_IMPORT_PREFIX}/lib/libDepthVistaSDK.so.1.0.10" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
