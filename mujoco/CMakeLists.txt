cmake_minimum_required(VERSION 3.0.2)
project(a1_mujoco)

aux_source_directory(./src MJ_SIM_LIST )
aux_source_directory(../src SRC_LIST )

include_directories( 
    ./include
    ../include
    ./mujoco210/include
) 


link_directories(./mujoco210/bin)
add_executable(a1_mujoco ${MJ_SIM_LIST} ./mujoco210/include/uitools.c 
            # ${SRC_LIST}
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

if (Eigen3_FOUND)
    message("########## The Eigen3 is ok ! ##########")
endif (Eigen3_FOUND)

if (pinocchio_FOUND)
    message("########## The pinocchio is ok ! ##########")
endif (pinocchio_FOUND)


include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${pinocchio_INCLUDE_DIRS})

target_link_libraries(a1_mujoco ${Eigen3_LIBRARIES})
target_link_libraries(a1_mujoco ${pinocchio_LIBRARIES})

target_link_libraries(a1_mujoco libglew.so)
target_link_libraries(a1_mujoco libglewegl.so)
target_link_libraries(a1_mujoco libglewosmesa.so)
target_link_libraries(a1_mujoco libmujoco210.so)
target_link_libraries(a1_mujoco libmujoco210nogl.so)
target_link_libraries(a1_mujoco libglfw.so.3)
target_link_libraries(a1_mujoco -lpthread -lGL -lglew)




