cmake_minimum_required(VERSION 3.16)

set(HOMEKIT_PATH ${CMAKE_CURRENT_LIST_DIR}/esp-homekit-sdk)

set(EXTRA_COMPONENT_DIRS ${HOMEKIT_PATH}/components ${HOMEKIT_PATH}/components/homekit )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(homekit-dev)
