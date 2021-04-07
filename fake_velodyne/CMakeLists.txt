cmake_minimum_required(VERSION 3.5)
project(rqt_fake_velodyne)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# further dependencies manually.
find_package(Qt5Widgets REQUIRED)

set(rqt_fake_velodyne_SRCS
  src/rqt_fake_velodyne/rqt_fake_velodyne.cpp
  src/rqt_fake_velodyne/rqt_fake_velodyne_widget.cpp
)

set(rqt_fake_velodyne_HDRS
  include/rqt_fake_velodyne/rqt_fake_velodyne.h
  include/rqt_fake_velodyne/rqt_fake_velodyne_widget.h
)

set(rqt_fake_velodyne_moc_HDRS
  include/rqt_fake_velodyne/rqt_fake_velodyne_widget.h
)

set(rqt_fake_velodyne_UIS
  ui/rqt_fake_velodyne/rqt_fake_velodyne_widget.ui
)

set(rqt_fake_velodyne_RS
  fake_velodyne.qrc
)

set(rqt_fake_velodyne_INCLUDE_DIRECTORIES
  include
  include/rqt_fake_velodyne
  ${rqt_gui_cpp_INCLUDE_DIRS}
)

file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")

qt5_wrap_cpp(rqt_fake_velodyne_MOCS ${rqt_fake_velodyne_moc_HDRS})
qt5_wrap_ui(rqt_fake_velodyne_UIS_H ${rqt_fake_velodyne_UIS})
qt5_add_resources (rqt_fake_velodyne_RS_H ${rqt_fake_velodyne_RS})

include_directories(${rqt_fake_velodyne_INCLUDE_DIRECTORIES} ${QWT_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED ${rqt_fake_velodyne_SRCS} ${rqt_fake_velodyne_MOCS} ${rqt_fake_velodyne_UIS_H} ${rqt_fake_velodyne_RS_H})
target_link_libraries(${PROJECT_NAME} ${QWT_LIBRARIES} ${rqt_gui_cpp_LIBRARIES} Qt5::Widgets cpprest crypto)

#############
## Install ##
#############

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
