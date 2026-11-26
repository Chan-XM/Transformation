# 位姿变换回归测试

add_executable(TransformationTest transformation_test.cpp)

target_link_libraries(TransformationTest
	PRIVATE TransformationCore
)

transformation_copy_runtime_dlls(TransformationTest)

# 注册到 CTest：程序退出码即失败用例个数，0 表示全部通过
add_test(NAME pose_transform_regression COMMAND TransformationTest)
set_tests_properties(pose_transform_regression PROPERTIES
	TIMEOUT 300
)
