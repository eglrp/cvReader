cmake_minimum_required(VERSION 2.8.4)
project(cvReader)

#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11   -pthread -lopencv_aruco")
#
#list(APPEND CMAKE_MODULE_PATH usr/share/OpenCV/
#        )
#set(OPENCV_DIR /opt/ros/jada/share/OpenCV-3.1.0-dev/)

find_package(Eigen3 REQUIRED)
#find_package(OPENCV REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})


include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include

)

link_directories(
        /usr/local/lib
        /usr/lib
        ${PCL_LIBRARY_DIRS}
)

add_executable(testcv src/main.cpp)
target_link_libraries(testcv ${OpenCV_LIBS})
#target_link_libraries(testcv ${OpenCV_LIBRARIES})

add_executable(gpu_test src/gpu-basics-similarity.cpp)
target_link_libraries(gpu_test ${OpenCV_LIBS})


add_executable(KF_test src/KF_test.cpp)
target_link_libraries(KF_test ${OpenCV_LIBS})


add_executable(Search src/enemy_search_main.cpp)
target_link_libraries(Search ${OpenCV_LIBS})

add_executable(generate_marker src/create_marker.cpp)
target_link_libraries(generate_marker ${OpenCV_LIBS})
#add_executable(real_time_pose_estimation real_time_pose_estimation/src/main_detection.cpp)
#target_link_libraries(real_time_pose_estimation ${OpenCV_LIBS})
add_executable(multikf src/MultiThreadKF.cpp)
target_link_libraries(multikf ${OpenCV_LIBS})


add_executable(DetectorMethodTest src/DetecterMethondTest.cpp)
target_link_libraries(DetectorMethodTest ${OpenCV_LIBS})


add_executable(Test_search src/thrid_main.cpp)
target_link_libraries(Test_search ${OpenCV_LIBS} ${PCL_LIBRARIES})
