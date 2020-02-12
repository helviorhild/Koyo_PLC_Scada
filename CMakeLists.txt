cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)
project(conect_plc)
#LIBS += -lwt -lwthttp -lboost_signals  -lpqxx -lpq -lopencv_highgui
add_executable(${PROJECT_NAME} "main.cpp" "plc_connect.cpp" "fluid/gui1.cxx")
target_link_libraries(${PROJECT_NAME} boost_system boost_chrono boost_thread pthread fltk Xext X11 m)
