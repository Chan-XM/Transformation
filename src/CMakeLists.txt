# 位姿变换核心库（静态库）：Eigen 与 OpenCV 两套实现

add_library(TransformationCore STATIC
	transformation_eg.cpp
	transformation_cv.cpp
)

# 头文件目录对外公开：demo 与 tests 直接 #include "transformation_eg.h" 即可
target_include_directories(TransformationCore PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(TransformationCore
	PUBLIC eigen_headers
	PUBLIC ${OpenCV_LIBS}
)

# 头文件列入 target，便于在 Visual Studio 解决方案资源管理器中看到
target_sources(TransformationCore
	PRIVATE
		common.h
		transformation_eg.h
		transformation_cv.h
)
