#===============================================================================
#
# Copyright (C) 2022Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-hand-viz)

find_package(RobotsIO REQUIRED)

find_package(RobotsViz REQUIRED)

find_package(YARP REQUIRED COMPONENTS
             cv    # seems that RobotsIO export is not working on Ubuntu18.04
             eigen # seems that RobotsIO export is not working on Ubuntu18.04
)

set(${EXE_TARGET_NAME}_HDR)

set(${EXE_TARGET_NAME}_SRC
    src/Viewer.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME}
                      RobotsIO::RobotsIO
                      RobotsViz::RobotsViz
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
