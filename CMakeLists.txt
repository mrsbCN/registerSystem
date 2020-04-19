cmake_minimum_required(VERSION 3.15)
project(register)

# 指定c++标准的版本
set(CMAKE_CXX_STANDARD 14)
# 自动调用moc，uic，rcc处理qt的扩展部分
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 设置Qt5的cmake模块所在目录，如果不设置将使用系统提供的版本
# QT_DIR和QT_VERSION是指定了qt安装目录和版本的环境变量
# 如果你使用了系统的cmake，那么会优先使用系统提供模块，因为cmake会优先搜索CMAKE_SYSTEM_PREFIX_PATH
# 如果你不想让cmake优先搜索系统目录（会导致编译使用系统安装的qt而不是我们配置的），需要提示find_package命令
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/mrsb/Qt5.14.2/5.14.2/gcc_64/lib/cmake)
# halcon
set(halcon_include_dirs "/usr/local/halcon/include/")
set(halcon_libs "/usr/local/halcon/lib/x64-linux/libhalcon.so;/usr/local/halcon/lib/x64-linux/libhalconcpp.so")
# 找到对应的qt模块，名字为qmake中QT += <name>中的name首字母大写后加上Qt5前缀
# 例如core为QtCore，你也可以去${CMAKE_PREFIX_PATH}的目录中找到正确的模块名
# 如果不想使用系统qt，这样写（注意NO_DEFAULT_PATH参数，它会让find_package跳过系统目录的查找）：
#[[FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()]]
find_package(Qt5 COMPONENTS Gui Core Widgets Concurrent X11Extras REQUIRED)
find_package(PCL 1.10.1 REQUIRED)
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
list(APPEND Open3D_LIBRARIES dl)

include_directories(${halcon_include_dirs})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# 如果你想要使用系统自带的qt，这样写：
# find_package(Qt5Widgets REQUIRED)

# 将当前目录的所有源文件添加进变量
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRCFILE)

# 通常这样设置就可以，如果你的项目包含qrc文件，那么需要将它们单独添加进来
# 例如add_executable(test ${DIRS_SRCS} resources.qrc)
add_executable(register ${SRCFILE})

# 把对应Qt模块的库链接进程序
target_link_libraries(register PUBLIC Qt5::Gui Qt5::Core Qt5::Widgets Qt5::Concurrent Qt5::X11Extras ${PCL_LIBRARIES} ${Open3D_LIBRARIES} ${halcon_libs})