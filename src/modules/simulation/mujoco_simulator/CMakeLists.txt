
#include(FetchContent)
#FetchContent_Declare(
#    mujoco
#    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
#    GIT_TAG 3.3.3 # Or a specific commit hash
#    SOURCE_DIR ${CMAKE_BINARY_DIR}/mujoco_src
#    #CONFIGURE_COMMAND ${CMAKE_COMMAND} -S <SOURCE_DIR> -B <BINARY_DIR>
#    #    -DCMAKE_BUILD_TYPE=Release
#    #    -DMUJOCO_BUILD_STATIC_LIBS=ON
#    #    -DMUJOCO_BUILD_TESTS=OFF
#    #    -DMUJOCO_BUILD_EXAMPLES=OFF
#    #    -DMUJOCO_USE_GL=OFF
#    #    -DCMAKE_C_FLAGS="-Wno-error=bad-function-cast -Wno-error=float-equal"
#    #    -DCMAKE_CXX_FLAGS="-Wno-error=bad-function-cast -Wno-error=float-equal"
#    #BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>
#    #    -- -Wno-error=bad-function-cast -Wno-error=float-equal
#)
#FetchContent_GetProperties(mujoco)
#if(NOT mujoco_POPULATED)
#	FetchContent_Populate(mujoco)
#	add_library(mujoco SHARED)
#	message(STATUS "mujoco build ${CMAKE_CURRENT_SOURCE_DIR}")
#	message(STATUS "mujoco src ${mujoco_SOURCE_DIR}")
#	message(STATUS "mujoco src ${mujoco_BINARY_DIR}")
#	target_include_directories(
#		mujoco
#		PUBLIC
#		${mujoco_SOURCE_DIR}/include/mujoco/
#		${mujoco_SOURCE_DIR}/src/engine
#	)
#	cmake_policy(SET CMP0076 NEW)
#	cmake_policy(SET CMP0079 NEW)
#
#	add_subdirectory(${mujoco_SOURCE_DIR}/plugin/elasticity ${mujoco_BINARY_DIR}/elasticity)
#	add_subdirectory(${mujoco_SOURCE_DIR}/plugin/actuator ${mujoco_BINARY_DIR}/actuator)
#	add_subdirectory(${mujoco_SOURCE_DIR}/plugin/sensor ${mujoco_BINARY_DIR}/sensor)
#	add_subdirectory(${mujoco_SOURCE_DIR}/plugin/sdf ${mujoco_BINARY_DIR}/sdf)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/engine ${mujoco_BINARY_DIR}/engine)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/user ${mujoco_BINARY_DIR}/user)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/xml ${mujoco_BINARY_DIR}/xml)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/render ${mujoco_BINARY_DIR}/render)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/thread ${mujoco_BINARY_DIR}/thread)
#	add_subdirectory(${mujoco_SOURCE_DIR}/src/ui ${mujoco_BINARY_DIR}/ui)
#	target_compile_definitions(mujoco PRIVATE _GNU_SOURCE CCD_STATIC_DEFINE MUJOCO_DLL_EXPORTS -DMC_IMPLEM_ENABLE)
#	target_compile_definitions(mujoco PUBLIC mjUSEPLATFORMSIMD)
#	target_compile_options(
#		mujoco
#		PRIVATE ${mujoco_AVX_COMPILE_OPTIONS}
#		${mujoco_MUJOCO_MACOS_COMPILE_OPTIONS}
#		${mujoco_EXTRA_COMPILE_OPTIONS}
#		${mujoco_MUJOCO_CXX_FLAGS}
#	)
#endif()
#FetchContent_MakeAvailable(mujoco)

find_package(mujoco REQUIRED)

message(STATUS "MUJOCO AVX OPTIONS: ${mujoco_AVX_COMPILE_OPTIONS}")
message(STATUS "MUJOCO EXTRA OPTIONS: ${mujoco_EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")



file(GLOB mj_models ${PX4_SOURCE_DIR}/Tools/simulation/mujoco/models/*)
file(GLOB mj_airframes ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_mujoco_*)

message(STATUS "models: ${mj_models}")
message(STATUS "airframes: ${mj_airframes}")

# Defines the mujoco_simulator module

#add_compile_options(-Wno-deprecated-declarations -Wno-address-of-packed-member)

# MJ_ROOT_DIR and find_path are no longer needed as Mujoco is built from source
# if(DEFINED ENV{MJ_ROOT_DIR} )
#         set(MJ_ROOT_DIR "$ENV{MJ_ROOT_DIR}" )
# endif()

# find_path(MJ_INCLUDE_DIR
#         NAMES
#         mujoco.h
#         PATHS
#         ${MJ_ROOT_DIR}/include/mujoco
#         /usr/include/mujoco
#         /usr/local/include/mujoco
# )


# The module is now added directly after Mujoco is made available
px4_add_module(
    MODULE modules__simulation__mujoco_simulator
    MAIN mujoco_simulator
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}
	${MUJOCO_SIMULATE_COMPILE_OPTIONS}
    SRCS
        MujocoSimulator.cpp
    DEPENDS
        mixer_module
        px4_work_queue
    MODULE_CONFIG
        module.yaml
)
set_property(TARGET modules__simulation__mujoco_simulator PROPERTY CXX_STANDARD 17)

target_link_libraries(modules__simulation__mujoco_simulator PRIVATE
	 #mujoco::mujoco
	 "/usr/local/lib/libmujoco.so"
)
px4_add_git_submodule(TARGET git_mujoco_bridge PATH "${PX4_SOURCE_DIR}/Tools/simulation/mujoco/mujoco_bridge")

# Extract model names from mj_models
set(mujoco_model_names)
foreach(model_path IN LISTS mj_models)
	get_filename_component(model_name ${model_path} NAME)
	list(APPEND mujoco_model_names ${model_name})
endforeach()

# Extract airframe names from mj_airframes and filter
set(models)
foreach(airframe_path IN LISTS mj_airframes)
	# Remove .post files
	if(airframe_path MATCHES "\.post$")
		continue()
	endif()

	string(REGEX REPLACE ".*_mujoco_([a-zA-Z0-9_]+)$" "\\1" airframe_model_name ${airframe_path})

	# Check if the airframe model name exists in the mujoco_model_names list
	list(FIND mujoco_model_names ${airframe_model_name} _index)
	if(_index GREATER -1)
		list(APPEND models ${airframe_model_name})
	else()
		message(WARNING "Airframe ${airframe_path} has no corresponding MuJoCo model directory.")
	endif()
endforeach()

list(REMOVE_DUPLICATES models)
set_property(GLOBAL PROPERTY PX4_MUJOCO_MODELS ${models})
message(STATUS "Generated MuJoCo models: ${models}")

configure_file(mujoco_env.sh.in ${PX4_BINARY_DIR}/rootfs/mujoco_env.sh)

foreach(model ${models})

	# match model to airframe
	set(airframe_model_only)
	set(airframe_sys_autostart)
	set(mujoco_airframe_found)
	foreach(mujoco_airframe IN LISTS mj_airframes)

		string(REGEX REPLACE ".*_mujoco_" "" airframe_model_only ${mujoco_airframe})
		#string(REGEX REPLACE "_mujoco_.*" "" airframe_sys_autostart ${mujoco_airframe})
		string(REGEX REPLACE ".*/([0-9]+)_mujoco_.*" "\\1" airframe_sys_autostart ${mujoco_airframe})

		if(model STREQUAL ${airframe_model_only})
			set(mujoco_airframe_found ${mujoco_airframe})
			break()
		endif()
	endforeach()

	if(mujoco_airframe_found)
		message(STATUS "mujoco model: ${model}")
		message(STATUS "airframe: ${airframe_model_only}, airframe:
		${mujoco_airframe_found}")
		message(STATUS "SYS_AUTOSTART: ${airframe_sys_autostart}")
	else()
		message(WARNING "mujoco missing model: ${model}")
	endif()


	message(STATUS "custom command: ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=mujoco_${model} MUJOCO_MODEL=${model} $<TARGET_FILE:px4>")
	add_custom_target(mujoco_${model}
		COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=mujoco_${model} MUJOCO_MODEL=${model} $<TARGET_FILE:px4>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4
	)
endforeach()
