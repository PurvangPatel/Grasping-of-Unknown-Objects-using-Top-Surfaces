# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Grasping-of-Unknown-Objects-using-Top-Surfaces_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Grasping-of-Unknown-Objects-using-Top-Surfaces_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Grasping-of-Unknown-Objects-using-Top-Surfaces_FOUND FALSE)
  elseif(NOT Grasping-of-Unknown-Objects-using-Top-Surfaces_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Grasping-of-Unknown-Objects-using-Top-Surfaces_FOUND FALSE)
  endif()
  return()
endif()
set(_Grasping-of-Unknown-Objects-using-Top-Surfaces_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Grasping-of-Unknown-Objects-using-Top-Surfaces_FIND_QUIETLY)
  message(STATUS "Found Grasping-of-Unknown-Objects-using-Top-Surfaces: 0.0.0 (${Grasping-of-Unknown-Objects-using-Top-Surfaces_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Grasping-of-Unknown-Objects-using-Top-Surfaces' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Grasping-of-Unknown-Objects-using-Top-Surfaces_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Grasping-of-Unknown-Objects-using-Top-Surfaces_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Grasping-of-Unknown-Objects-using-Top-Surfaces_DIR}/${_extra}")
endforeach()
