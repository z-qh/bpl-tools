cmake_minimum_required(VERSION 3.16)

project(camera_driver)


find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	)



add_executable(camera_driver src/camera_driver.cc)

target_link_libraries(camera_driver
	${OpenCV_LIBS}
	)
	
	
add_executable(camera_rec src/camera_rec.cc)

target_link_libraries(camera_rec
	${OpenCV_LIBS}
	)
	
