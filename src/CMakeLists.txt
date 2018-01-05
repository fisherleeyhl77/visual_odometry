# Find necessary packages
find_package(PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(OpenCV 2.4.9 REQUIRED)

# PCL dependencies
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

# Add executables
file(GLOB SOURCES "./*.cpp")
add_executable(visual_odometry ${SOURCES})
target_link_libraries(visual_odometry ${OpenCV_LIBS} ${PCL_LIBRARIES})
