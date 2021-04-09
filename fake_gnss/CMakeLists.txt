cmake_minimum_required(VERSION 3.5)
project(rqt_fake_gnss)

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
find_package(Boost REQUIRED COMPONENTS
  thread
)

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
  ${rqt_gui_cpp_INCLUDE_DIRS}
)

file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")

qt5_wrap_cpp(rqt_fake_gnss_MOCS ${rqt_fake_gnss_moc_HDRS})
qt5_wrap_ui(rqt_fake_gnss_UIS_H ${rqt_fake_gnss_UIS})
qt5_add_resources (rqt_fake_gnss_RS_H ${rqt_fake_gnss_RS})

include_directories(${rqt_fake_gnss_INCLUDE_DIRECTORIES} ${QWT_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED ${rqt_fake_gnss_SRCS} ${rqt_fake_gnss_MOCS} ${rqt_fake_gnss_UIS_H} ${rqt_fake_gnss_RS_H})
target_link_libraries(${PROJECT_NAME} ${QWT_LIBRARIES} ${rqt_gui_cpp_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets)

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
