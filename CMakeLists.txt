cmake_minimum_required (VERSION 2.8)
project (Ximea)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

FIND_PACKAGE(OpenCV REQUIRED)
find_package(Torch REQUIRED)


include_directories("${PROJECT_SOURCE_DIR}/include" ${Torch_INSTALL_INCLUDE})
include_directories("/opt/XIMEA/include/")


link_libraries(m3api ${Torch_INSTALL_LIB} )

add_library(Ximea SHARED src/ximeawrapper.cpp)
#add_executable(XimeaReader src/ximeawrapper.cpp)
target_link_libraries(Ximea ${OpenCV_LIBS})
#target_link_libraries(XimeaReader ${OpenCV_LIBS} m3api ${Torch_INSTALL_LIB})


install(DIRECTORY "lua/" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/ximea" FILES_MATCHING PATTERN "*.lua")

install(TARGETS Ximea
        RUNTIME DESTINATION bin COMPONENT main
        LIBRARY DESTINATION lib COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)
