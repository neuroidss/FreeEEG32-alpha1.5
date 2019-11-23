CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
cmake_policy(SET CMP0026 OLD)
cmake_policy(SET CMP0042 OLD)
cmake_policy(SET CMP0043 OLD)
cmake_policy(SET CMP0045 OLD)
cmake_policy(SET CMP0048 OLD)

PROJECT(OpenVIBE)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
SET(CMAKE_INSTALL_LIBDIR lib) # Fix for non debian-based distrib, as it seeems to be a common occurence to name this folder lib32/lib64
INCLUDE("Utilities")
INCLUDE("SetProjectVersion")
set_version()
OV_SET_PROJECT_VERSION()
MESSAGE(STATUS "BUILDING VERSION OpenViBE SDK: ${OV_GLOBAL_VERSION_STRING} Branch : ${OV_PROJECT_BRANCH}~${OV_PROJECT_COMMITHASH}")

# Default is to build to dist/. If you wish a custom install, set your own MAKE_INSTALL_PREFIX when you call CMake. Safest to do under a fakeroot.
MESSAGE(STATUS "Appending compilation flags...")

SET(OV_CONFIG_SUBDIR "${PROJECT_NAME}" CACHE STRING "Subdirectory under user directory when configuration and logs will be saved")

IF(NOT DEFINED OV_CUSTOM_DEPENDENCIES_PATH)
	IF(WIN32)
		IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
			SET(OV_DEPENDENCIES_PLATFORM_FOLDER_NAME "dependencies_x64")
		ELSEIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
			SET(OV_DEPENDENCIES_PLATFORM_FOLDER_NAME "dependencies")
		ELSE()
			SET(OV_DEPENDENCIES_PLATFORM_FOLDER_NAME "dependencies")
		ENDIF()
	ELSE()
		SET(OV_DEPENDENCIES_PLATFORM_FOLDER_NAME "dependencies")
	ENDIF()
ENDIF()

SET(OV_CUSTOM_DEPENDENCIES_PATH "${CMAKE_SOURCE_DIR}/${OV_DEPENDENCIES_PLATFORM_FOLDER_NAME}" CACHE STRING "Dependency directory")
SET(OV_SOURCE_DEPENDENCIES_PATH "${CMAKE_SOURCE_DIR}/dependencies-source")

# ----------------------
# Build CMake flags
# ----------------------
SET(BRAND_NAME "OpenViBE" CACHE STRING "Brand with which titles and documentation will be prefixed")
string(TIMESTAMP COPYRIGHT_DATE "2012-%Y")
IF(BRAND_NAME STREQUAL "NeuroRT")
	SET(COMPANY_NAME "Mensia Technologies SA")
ELSE()
	SET(COMPANY_NAME "Inria")
ENDIF()

OPTION(Flag_VerboseOutput "Verbose CMake output" OFF)
OPTION(OV_DISPLAY_ERROR_LOCATION ON)
OPTION(PUBLISH_DOC_ASSETS "Publish plugin documentation assets" ON)

if(NOT(${Flag_VerboseOutput}))
	set(CMAKE_INSTALL_MESSAGE LAZY)
endif()

# If OpenMP is available, using it will allow Eigen to use multiple cores in matrix math.
IF(NOT(APPLE))
	INCLUDE("FindOpenMP")
	IF(OPENMP_FOUND)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	ENDIF(OPENMP_FOUND)
ENDIF()

ADD_DEFINITIONS("-DOV_CONFIG_SUBDIR=\"${OV_CONFIG_SUBDIR}\"")
ADD_DEFINITIONS("-DOV_PROJECT_NAME=\"${PROJECT_NAME}\"")
ADD_DEFINITIONS("-DOV_VERSION_MAJOR=\"${OV_GLOBAL_VERSION_MAJOR}\"")
ADD_DEFINITIONS("-DOV_VERSION_MINOR=\"${OV_GLOBAL_VERSION_MINOR}\"")
ADD_DEFINITIONS("-DOV_VERSION_PATCH=\"${OV_GLOBAL_VERSION_PATCH}\"")

IF(NOT DEFINED OV_PACKAGE)
	SET(OV_PACKAGE FALSE)
ENDIF(NOT DEFINED OV_PACKAGE)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
IF(WIN32)
	ADD_DEFINITIONS("-DNOMINMAX -DBOOST_ALL_NO_LIB")
	# The following is needed to keep OpenMP from hogging all the cores. Note that this works only for VS2013+. On VS2010, it may be better to disable OpenMP.
	SET(OV_OMP_WAIT_POLICY "PASSIVE")
	# Disable /MP if you don't like VS using all the cores for compilation
	
	# Switch /arch:SSE2 enables vectorization. Remove if your CPU/compiler doesn't support it.
	# x64 build do not provide SSE2 option because 64bit processors always supports SSE vectorization.
	# So the /arch:SSE2 is not recognized for x64 build we have to only activate it for x86.
	IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
	ELSEIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
#		SET(OV_EIGEN_FLAGS "/arch:SSE2")
	ENDIF()
	
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4355 /MP ${OV_EIGEN_FLAGS}")
	SET(OV_WIN32_BOOST_VERSION "1_58")
ELSEIF(APPLE)
#	SET(OV_EIGEN_FLAGS "-msse2")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${OV_EIGEN_FLAGS}")
ELSEIF(UNIX)
	# Switch -msse2 enables vectorization. Remove if your CPU/compiler doesn't support it.
#	SET(OV_EIGEN_FLAGS "-msse2")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${OV_EIGEN_FLAGS}")
ELSE()
	MESSAGE(WARNING "Warning: unknown platform")
ENDIF()

IF(NOT CMAKE_INSTALL_PREFIX)
	SET(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../openvibe-sdk-build/dist-${CMAKE_BUILD_TYPE}")
ENDIF()

# Setup knowledge of GNU-style install path variables
INCLUDE("GNUInstallDirs")

if(NOT CMAKE_BUILD_TYPE AND CMAKE_GENERATOR MATCHES "Visual Studio*")
	set(MULTI_BUILD TRUE)
	set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin" )
	set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/lib" )
	# set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/archive" )
	foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
		string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIGU )
		set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIGU} "${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}/bin" )
		set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIGU} "${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}/lib" )
		# set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}/archive" )
		string(CONCAT DIST_ROOT ${DIST_ROOT} $<$<CONFIG:${OUTPUTCONFIGU}>:${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}>)
	endforeach()
	set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "OV_CMAKE_PATH_ROOT=\"${DIST_ROOT}\"")
	set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "OV_CMAKE_PATH_BIN=\"${DIST_ROOT}/bin\"")
	set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "OV_CMAKE_PATH_LIB=\"${DIST_ROOT}/lib\"")
	set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "OV_CMAKE_PATH_DATA=\"${DIST_ROOT}/share/openvibe\"")
	set(DIST_BINDIR ${DIST_ROOT}/bin)
	set(DIST_LIBDIR ${DIST_ROOT}/lib)
	set(DIST_INCLUDEDIR ${DIST_ROOT}/include)
	set(DIST_DATADIR ${DIST_ROOT}/share)
	set(DIST_DOCDIR ${DIST_ROOT}/doc)
	set(DIST_DOCTMP ${DIST_ROOT}/doc-tmp)
	
	set(DIST_BINDIR_ABSOLUTE ${DIST_BINDIR})
	# Also install it on the base dir for convenience
	install(FILES scripts/AddOpenViBESDKComponents.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share)
	# Install the .install file inside the dists folders
	install(FILES ${CMAKE_BINARY_DIR}/.install DESTINATION ${DIST_ROOT})
elseif(CMAKE_BUILD_TYPE AND OV_PACKAGE)
	# Add directories that OpenViBE will use to look for its components runtime, unless overridden by environment variables in the launch scripts.
	# These defines should only be used in "openvibe/ov_directories.h".

	# We use absolute paths to avoid issue for unit and validation tests
	get_filename_component(DIST_ROOT_ABSOLUTE "." REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
	set(DIST_BINDIR_ABSOLUTE ${DIST_ROOT_ABSOLUTE}/bin)

	set(CMAKE_INSTALL_PREFIX .)

	add_definitions(-DOV_CMAKE_PATH_ROOT="${DIST_ROOT_ABSOLUTE}")
	add_definitions(-DOV_CMAKE_PATH_BIN="${DIST_ROOT_ABSOLUTE}/bin")
	add_definitions(-DOV_CMAKE_PATH_LIB="${DIST_ROOT_ABSOLUTE}/lib")
	add_definitions(-DOV_CMAKE_PATH_DATA="${DIST_ROOT_ABSOLUTE}/share/openvibe")

	set(DIST_ROOT ${CMAKE_INSTALL_PREFIX})
	set(DIST_BINDIR ${CMAKE_INSTALL_BINDIR})
	set(DIST_LIBDIR ${CMAKE_INSTALL_LIBDIR})
	set(DIST_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
	set(DIST_DATADIR ${CMAKE_INSTALL_DATADIR})
	set(DIST_DOCDIR doc)
	set(DIST_DOCTMP doc-tmp)
	set(CPACK_GENERATOR "ZIP")
	set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
	set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${BRAND_NAME})
	set(CPACK_PACKAGE_VENDOR "Inria & Mensia Technologies")
	set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
	set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING.md")
	set(CPACK_PACKAGE_VERSION ${OV_GLOBAL_VERSION_MAJOR}.${OV_GLOBAL_VERSION_MINOR}.${OV_GLOBAL_VERSION_PATCH}.${OV_GLOBAL_VERSION_BUILD})
	set(CPACK_PACKAGE_VERSION_MAJOR ${OV_GLOBAL_VERSION_MAJOR})
	set(CPACK_PACKAGE_VERSION_MINOR ${OV_GLOBAL_VERSION_MINOR})
	set(CPACK_PACKAGE_VERSION_PATCH ${OV_GLOBAL_VERSION_PATCH})

	if(UNIX)
		execute_process(
			COMMAND bash -c "echo -n $(lsb_release -i -s)$(lsb_release -r -s)"
			OUTPUT_VARIABLE CPACK_SYSTEM_NAME)
	endif()

	set(CPACK_PACKAGE_NAME "OpenViBE-${CMAKE_BUILD_TYPE}")
	include(CPack)
elseif(CMAKE_BUILD_TYPE)
	# Add directories that OpenViBE will use to look for its components runtime, unless overridden by environment variables in the launch scripts.
	# These defines should only be used in "openvibe/ov_directories.h".
	add_definitions(-DOV_CMAKE_PATH_ROOT="${CMAKE_INSTALL_PREFIX}")
	add_definitions(-DOV_CMAKE_PATH_BIN="${CMAKE_INSTALL_PREFIX}/bin")
	add_definitions(-DOV_CMAKE_PATH_LIB="${CMAKE_INSTALL_PREFIX}/lib")
	add_definitions(-DOV_CMAKE_PATH_DATA="${CMAKE_INSTALL_PREFIX}/share/openvibe")
	
	set(DIST_ROOT ${CMAKE_INSTALL_PREFIX})
	set(DIST_BINDIR ${CMAKE_INSTALL_FULL_BINDIR})
	set(DIST_LIBDIR ${CMAKE_INSTALL_FULL_LIBDIR})
	set(DIST_INCLUDEDIR ${CMAKE_INSTALL_FULL_INCLUDEDIR})
	set(DIST_DATADIR ${CMAKE_INSTALL_FULL_DATADIR})
	set(DIST_DOCDIR ${DIST_ROOT}/doc)
	set(DIST_DOCTMP ${DIST_ROOT}/doc-tmp)
	
	set(DIST_BINDIR_ABSOLUTE ${DIST_BINDIR})
else()
	message(FATAL_ERROR "Build should specify a type or use a multi-type generator (like Visual Studio)")
endif()

# Print the used compilation parameters (for transparency)
GET_DIRECTORY_PROPERTY(TMP_DEFINITIONS COMPILE_DEFINITIONS)
MESSAGE(STATUS "Compilation flags used at source root: ")
MESSAGE(STATUS "  COMPILE_DEFINITIONS = '${TMP_DEFINITIONS}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS = '${CMAKE_CXX_FLAGS}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS_RELEASE = '${CMAKE_CXX_FLAGS_RELEASE}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS_DEBUG = '${CMAKE_CXX_FLAGS_DEBUG}'")

FUNCTION(SET_BUILD_PLATFORM)
	IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_x64)
	ELSEIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_i386)
		SET(PLATFORM_TARGET "x86")
	ELSE()
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_Unknown)
	ENDIF()

	IF(WIN32)
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
		ADD_DEFINITIONS(-DTARGET_OS_Windows)
		ADD_DEFINITIONS(-DTARGET_COMPILER_VisualStudio)
	ELSEIF(APPLE)
		ADD_DEFINITIONS(-fnon-call-exceptions)
		ADD_DEFINITIONS(-DTARGET_OS_MacOS)
		# ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_x64)
		ADD_DEFINITIONS(-DTARGET_COMPILER_LLVM)
	ELSEIF(UNIX)
		# ADD_DEFINITIONS(-fvisibility=hidden) # This flag should be present... man gcc
		ADD_DEFINITIONS(-fnon-call-exceptions)
		ADD_DEFINITIONS(-DTARGET_OS_Linux)
		ADD_DEFINITIONS(-DTARGET_COMPILER_GCC)
	ENDIF()
ENDFUNCTION()

# Custom cmakelist can be used to overwrite the default compilation & packaging parameters
# e.g. OEM_DISTRIBUTION
IF(EXISTS "${CMAKE_SOURCE_DIR}/CustomCMakeLists.txt")
	MESSAGE(STATUS "Found custom build settings")
	INCLUDE("${CMAKE_SOURCE_DIR}/CustomCMakeLists.txt")
ENDIF()

#################################################################
string(TOLOWER ${BRAND_NAME} LOWER_BRAND_NAME)
ADD_DEFINITIONS(-DBRAND_NAME="${BRAND_NAME}")
if(${BRAND_NAME} STREQUAL "NeuroRT")
	ADD_DEFINITIONS(-DOV_LOCAL_SYMBOLS)
endif()

IF(OV_DISPLAY_ERROR_LOCATION)
	ADD_DEFINITIONS(-DOV_DISPLAY_ERROR_LOCATION)
ENDIF()
#################################################################

SET(OV_LAUNCHER_SOURCE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules/launchers")

# By setting SKIP[_FOLDER]* you can skip a subtree (example: SKIP_A_B_C skips folder a/b/c and all its subfolders if any)
# Skip building documentation
# a list of all project which will be filled by the directory traversal. This is needed to generate the documentation.
SET_PROPERTY(GLOBAL PROPERTY OV_PROP_CURRENT_PROJECTS "")
SET_PROPERTY(GLOBAL PROPERTY OV_PROP_CURRENT_PROJECTS_BUILD_DIR "")

# Used by the various Find* scripts to locate OpenViBE modules
SET(OV_BASE_DIR ${CMAKE_SOURCE_DIR})
SET(OV_BASE_BIN_DIR ${CMAKE_BINARY_DIR})
SET(OV_LIBRARY_DIR  ${CMAKE_BINARY_DIR}/lib)

# needed for making visual studio projects when this script is called without CMAKE_BUILD_TYPE
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG "TARGET_BUILDTYPE_Debug")
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE "TARGET_BUILDTYPE_Release")
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELWITHDEBINFO "TARGET_BUILDTYPE_Release")

# OpenViBE-specific helper functions that are used by the CMakeLists of the subprojects.
INCLUDE("OvAddProjects")
INCLUDE("OvInstallLaunchScript")
INCLUDE("OvLinkBoostLib")
INCLUDE("OvDelayLoad")

# Sets the PROJECT_VERSION variable to something, depending on overriding OvSetProjectVersion.cmake files
OV_SET_PROJECT_VERSION()

# Traverse these directories and build their components
ADD_SUBDIRECTORY("build-tool/")

ADD_SUBDIRECTORY("documentation/")
ADD_SUBDIRECTORY("common/")

IF(NOT(SKIP_OPENVIBE))
    ADD_SUBDIRECTORY("openvibe/")
ENDIF()

IF(NOT(SKIP_KERNEL))
    ADD_SUBDIRECTORY("kernel/")
ENDIF()

ADD_SUBDIRECTORY("modules/")

IF(NOT(SKIP_TOOLKIT))
    ADD_SUBDIRECTORY("toolkit/")
ENDIF()

IF(NOT(SKIP_PLUGINS))
    ADD_SUBDIRECTORY("plugins/")
ENDIF()

IF(NOT(SKIP_APPLICATIONS))
    ADD_SUBDIRECTORY("applications/")
ENDIF()

INSTALL(FILES "${CMAKE_BINARY_DIR}/.install" DESTINATION ${CMAKE_INSTALL_PREFIX})


OPTION(BUILD_TESTING "Include and build unit tests" ON)

IF(BUILD_TESTING)
	SET(OVT_LOG_DIR "${CMAKE_BINARY_DIR}/test/log" CACHE PATH "Path to directory containing test logs")
	SET(OVT_TEST_DATA_DIR "${OV_CUSTOM_DEPENDENCIES_PATH}/test-input" CACHE PATH "Path to test input data files")
ENDIF()

# add the scripts to the project so IDEs using the CMake file are aware of them
FILE(GLOB_RECURSE script_files scripts/*.cmd scripts/*.sh scripts/*.nsi scripts/*.cmake)
INSTALL(FILES scripts/AddOpenViBESDKComponents.cmake DESTINATION ${DIST_DATADIR})
ADD_CUSTOM_TARGET(openvibe-scripts SOURCES ${script_files})

# Enable/Disable unit test
SET(BUILD_UNIT_TEST true CACHE BOOL "Set this flag to true/false to enable/disable unit test building")
SET(BUILD_VALIDATION_TEST true CACHE BOOL "Set this flag to true/false to enable/disable validation test building")

IF(BUILD_UNIT_TEST) 
	ADD_SUBDIRECTORY("unit-test/")
ENDIF()
IF(BUILD_VALIDATION_TEST) 
	ADD_SUBDIRECTORY("validation-test/")
ENDIF()
