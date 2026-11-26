# 使用示例（原 main.cpp 的手工验证程序）

add_executable(TransformationDemo main.cpp)

target_link_libraries(TransformationDemo
	PRIVATE TransformationCore
)

transformation_copy_runtime_dlls(TransformationDemo)
