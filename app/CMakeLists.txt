add_executable(shell-app main.cpp ImageManipulate.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
find_package( OpenCV REQUIRED )
find_package (Threads)
target_link_libraries(shell-app ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})