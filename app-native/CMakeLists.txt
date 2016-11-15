project(app-native)

# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

file(GLOB_RECURSE SRC_FILES src/main/cpp/*.cpp)
include_directories(src/main/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fcxx-exceptions -std=c++1y -Wformat=2 -Wall -Wextra -Werror -pedantic -Wno-unused-parameter")

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add.library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
             app-native

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             ${SRC_FILES} )

find_library( # Defines the name of the path variable that stores the
              # location of the NDK library.
              log-lib

              # Specifies the name of the NDK library that
              # CMake needs to locate.
              log )

# Links your native library against one or more other native libraries.
target_link_libraries( # Specifies the target library.
                       app-native

                       # Links the log library to the target library.
                       ${log-lib} )

add_library( libavcodec
             SHARED
             IMPORTED )

set_target_properties( # Specifies the target library.
                       libavcodec

                       # Specifies the parameter you want to define.
                       PROPERTIES IMPORTED_LOCATION

                       # Provides the path to the library you want to import.
                       /src/main/lib/libavcodec-57.so )

add_library( libavformat
             SHARED
             IMPORTED )

set_target_properties( # Specifies the target library.
                       libavformat

                       # Specifies the parameter you want to define.
                       PROPERTIES IMPORTED_LOCATION

                       # Provides the path to the library you want to import.
                       /src/main/lib/libavformat-57.so )

add_library( libopencv_java3
             SHARED
             IMPORTED )

set_target_properties( # Specifies the target library.
                       libopencv_java3

                       # Specifies the parameter you want to define.
                       PROPERTIES IMPORTED_LOCATION

                       # Provides the path to the library you want to import.
                       ../app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so )
