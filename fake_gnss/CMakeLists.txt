cmake_minimum_required(VERSION 3.0.2)
project(rqt_fake_gnss)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
)

## System dependencies are found with CMake's conventions
find_package(Qt5Widgets REQUIRED)


set(rqt_fake_gnss_SRCS
  src/rqt_fake_gnss/rqt_fake_gnss.cpp
  src/rqt_fake_gnss/rqt_fake_gnss_widget.cpp
)

set(rqt_fake_gnss_HDRS
  include/rqt_fake_gnss/rqt_fake_gnss.h
  include/rqt_fake_gnss/rqt_fake_gnss_widget.h
)

set(rqt_fake_gnss_moc_HDRS
  include/rqt_fake_gnss/rqt_fake_gnss_widget.h
)

set(rqt_fake_gnss_UIS
  ui/rqt_fake_gnss/rqt_fake_gnss_widget.ui
)

set(rqt_fake_gnss_RS
  fake_gnss.qrc
)

set(rqt_fake_gnss_INCLUDE_DIRECTORIES
  include
  include/rqt_fake_gnss
  ${CMAKE_CURRENT_BINARY_DIR}
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)

file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")

qt5_wrap_cpp(rqt_fake_gnss_MOCS ${rqt_fake_gnss_moc_HDRS})
qt5_wrap_ui(rqt_fake_gnss_UIS_H ${rqt_fake_gnss_UIS})
qt5_add_resources (rqt_fake_gnss_RS_H ${rqt_fake_gnss_RS})

include_directories(${rqt_fake_gnss_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${rqt_fake_gnss_SRCS} ${rqt_fake_gnss_MOCS} ${rqt_fake_gnss_UIS_H} ${rqt_fake_gnss_RS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QWT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml fake_gnss.qrc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images
)
