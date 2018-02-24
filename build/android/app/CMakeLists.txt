# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

file(GLOB Detour_Sources "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/Detour/Source/*.cpp")
file(GLOB DetourCrowd_Sources "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/DetourCrowd/Source/*.cpp")
file(GLOB Nav_Sources "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/Nav/Source/*.cpp")

file(GLOB NMGen_Sources "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nmgen-rcn/NMGen/Source/*.cpp")
file(GLOB Recast_Sources "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nmgen-rcn/Recast/Source/*.cpp")

include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/Detour/Include"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/DetourCrowd/Include"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nav-rcn/Nav/Include"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nmgen-rcn/NMGen/Include"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/nmgen-rcn/Recast/Include" )

add_library( # Sets the name of the library.
             cai-nav-rcn

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             ${Detour_Sources}
             ${DetourCrowd_Sources}
             ${Nav_Sources} )

add_library( # Sets the name of the library.
             cai-nmgen-rcn

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             ${NMGen_Sources}
             ${Recast_Sources} )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       cai-nav-rcn
                       cai-nmgen-rcn

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )