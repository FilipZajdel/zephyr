# SPDX-License-Identifier: Apache-2.0

if (NOT DEFINED ENV{BSIM_COMPONENTS_PATH})
    message(FATAL_ERROR "This board requires the BabbleSim simulator. Please set\
 the  environment variable BSIM_COMPONENTS_PATH to point to its components \
 folder. More information can be found in\
 https://babblesim.github.io/folder_structure_and_env.html")
endif()

if (NOT DEFINED ENV{BSIM_OUT_PATH})
    message(FATAL_ERROR "This board requires the BabbleSim simulator. Please set\
 the environment variable BSIM_OUT_PATH to point to the folder where the\
 simulator is compiled to. More information can be found in\
 https://babblesim.github.io/folder_structure_and_env.html")
endif()

zephyr_library()

zephyr_library_compile_definitions(NO_POSIX_CHEATS)

zephyr_library_sources(
	hw_models_top.c
	timer_model.c
	native_rtc.c
	irq_handler.c
	irq_ctrl.c
	main.c
	tracing.c
	cmdline_common.c
	cmdline.c
	hw_counter.c
)

zephyr_library_include_directories(
  $ENV{BSIM_COMPONENTS_PATH}/libUtilv1/src/
  $ENV{BSIM_COMPONENTS_PATH}/libPhyComv1/src/
  $ENV{BSIM_COMPONENTS_PATH}/libRandv2/src/
  $ENV{BSIM_COMPONENTS_PATH}/ext_2G4_libPhyComv1/src/
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/posix/include
)

if(CONFIG_USE_BABBLESIM)
  zephyr_library_sources(bs_radio.c
  						 bs_radio_argparse.c)

  zephyr_library_include_directories(
	$ENV{BSIM_COMPONENTS_PATH}/libUtilv1/src/
  	$ENV{BSIM_COMPONENTS_PATH}/libPhyComv1/src/
  	$ENV{BSIM_COMPONENTS_PATH}/libRandv2/src/
  	$ENV{BSIM_COMPONENTS_PATH}/ext_2G4_libPhyComv1/src/
  )

  set(bsim_lib_path $ENV{BSIM_OUT_PATH}/lib)
  zephyr_library_import(bsim_libUtilv1	  ${bsim_lib_path}/libUtilv1.32.a)
  zephyr_library_import(bsim_libPhyComv1    ${bsim_lib_path}/libPhyComv1.32.a)
  zephyr_library_import(bsim_lib2G4PhyComv1 ${bsim_lib_path}/lib2G4PhyComv1.32.a)
  zephyr_library_import(bsim_libRandv2      ${bsim_lib_path}/libRandv2.32.a)
endif()

if(CONFIG_HAS_SDL)
	find_package(PkgConfig REQUIRED)
	pkg_search_module(SDL2 REQUIRED sdl2)
	zephyr_include_directories(${SDL2_INCLUDE_DIRS})
	zephyr_link_libraries(${SDL2_LIBRARIES})
	zephyr_compile_options(${SDL2_CFLAGS_OTHER})
	zephyr_library_sources(sdl_events.c)
endif()

zephyr_ld_options(
  -lm
)


# Override the C standard used for compilation to C 2011
# This is due to some tests using _Static_assert which is a 2011 feature, but
# otherwise relying on compilers supporting it also when set to C99.
# This was in general ok, but with some host compilers and C library versions
# it led to problems. So we override it to 2011 for native_posix.
set_property(GLOBAL PROPERTY CSTD c11)
