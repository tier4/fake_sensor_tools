cmake_minimum_required(VERSION 3.5)
project(rqt_fake_livox)

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

set(rqt_fake_livox_SRCS
  src/rqt_fake_livox/fake_point_cloud.cpp
  src/rqt_fake_livox/rqt_fake_livox.cpp
  src/rqt_fake_livox/rqt_fake_livox_widget.cpp
  src/rqt_fake_livox/sdk_protocol.cpp
  src/rqt_fake_livox/udp_list_model.cpp
  src/third_party/FastCRC/FastCRC_tables.hpp
  src/third_party/FastCRC/FastCRCsw.cpp
)

set(rqt_fake_livox_HDRS
  include/rqt_fake_livox/rqt_fake_livox.h
  include/rqt_fake_livox/rqt_fake_livox_widget.h
)

set(rqt_fake_livox_moc_HDRS
  include/rqt_fake_livox/rqt_fake_livox_widget.h
)

set(rqt_fake_livox_UIS
  ui/rqt_fake_livox/rqt_fake_livox_widget.ui
)

set(rqt_fake_livox_RS
  fake_livox.qrc
)

set(rqt_fake_livox_INCLUDE_DIRECTORIES
  include
  include/rqt_fake_livox
  include/third_party/FastCRC
  ${rqt_gui_cpp_INCLUDE_DIRS}
)

file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")

qt5_wrap_cpp(rqt_fake_livox_MOCS ${rqt_fake_livox_moc_HDRS})
qt5_wrap_ui(rqt_fake_livox_UIS_H ${rqt_fake_livox_UIS})
qt5_add_resources(rqt_fake_livox_RS_H ${rqt_fake_livox_RS})

include_directories(${rqt_fake_livox_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED ${rqt_fake_livox_SRCS} ${rqt_fake_livox_MOCS} ${rqt_fake_livox_UIS_H} ${rqt_fake_livox_RS_H})
target_link_libraries(${PROJECT_NAME} ${QWT_LIBRARIES} ${rqt_gui_cpp_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets pcap)

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
