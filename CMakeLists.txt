cmake_minimum_required(VERSION 3.25)

project(Transformation
	VERSION 1.0
	DESCRIPTION "三维空间位姿与矩阵之间的变换（Eigen 与 OpenCV 双实现）"
	LANGUAGES CXX)

# ===========================================================================
# 全局编译设置
# ===========================================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(TRANSFORMATION_COPY_OPENCV_DLL "把 OpenCV 运行时 dll 复制到可执行文件目录" ON)

# 产物统一放到 build/bin 与 build/lib。
# 多配置生成器（Visual Studio）会自动再追加一层配置子目录，即 build/bin/Debug、build/lib/Release。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# ===========================================================================
# 依赖
# ===========================================================================
# Eigen：仅头文件，直接指向源码树即可，无需安装
set(EIGEN3_INCLUDE_DIR "E:/Package/Eigen/eigen-3.4.0" CACHE PATH "Eigen 头文件目录（含 Eigen/Dense 的那一层）")
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Dense")
	message(FATAL_ERROR
		"Eigen 未找到：${EIGEN3_INCLUDE_DIR}/Eigen/Dense 不存在。\n"
		"请重新配置并指定 -DEIGEN3_INCLUDE_DIR=<Eigen 源码根目录>")
endif()

add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers INTERFACE "${EIGEN3_INCLUDE_DIR}")

# OpenCV：使用预编译包自带的 OpenCVConfig.cmake，其导入目标已按 Debug/Release
# 分别指向 opencv_world470d.lib / opencv_world470.lib，无需手工区分 d 版本。
#
# 注意：该脚本会按 MSVC 版本反推预编译包目录名（如 vc16），推断值与目录名不符时
# 会直接判定为「找不到 OpenCV」。这里显式给出 arch / runtime，命中
# OpenCVConfig.cmake 里 "custom overridden values" 的分支，从而绕开版本推断：
# 本机预编译包由 VS2019(vc16) 构建；若将来换成与工具集匹配的新包，删掉下面两行即可。
set(OpenCV_ARCH "x64" CACHE STRING "OpenCV 预编译包的目标架构")
set(OpenCV_RUNTIME "vc16" CACHE STRING "OpenCV 预编译包的运行时标签（vc16 对应 VS2019）")
set(OpenCV_DIR "E:/installed/OpenCV/opencv_4.7.0_vc16/build" CACHE PATH "OpenCV 的 build 目录（内含 OpenCVConfig.cmake）")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION} -> ${OpenCV_DIR}  (arch=${OpenCV_ARCH}, runtime=${OpenCV_RUNTIME})")

# ===========================================================================
# 公共辅助函数
# ===========================================================================
# 静态库依赖的 dll 不会自动出现在可执行文件目录，这里显式复制，
# 使 build/bin/<Config> 下的程序可以直接双击运行
function(transformation_copy_runtime_dlls _target)
	if(WIN32 AND TRANSFORMATION_COPY_OPENCV_DLL)
		add_custom_command(TARGET ${_target} POST_BUILD
			COMMAND "${CMAKE_COMMAND}" -E copy_if_different
				"$<TARGET_RUNTIME_DLLS:${_target}>" "$<TARGET_FILE_DIR:${_target}>"
			COMMAND_EXPAND_LISTS
			VERBATIM)
	endif()
endfunction()

# ===========================================================================
# 子目录
# ===========================================================================
include(CTest)			# 提供 BUILD_TESTING 选项并 enable_testing()

add_subdirectory(src)	# 位姿变换核心库
add_subdirectory(demo)	# 使用示例

if(BUILD_TESTING)
	add_subdirectory(tests)	# 回归测试
endif()
