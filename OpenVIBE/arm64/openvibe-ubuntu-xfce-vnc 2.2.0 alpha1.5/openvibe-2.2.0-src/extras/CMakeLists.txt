CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

# Here we set a few policies to get rid of warnings on newer cmakes.
# we should update the scripts after upgrading each platform's cmake to 3.x.
cmake_policy(SET CMP0026 OLD)
cmake_policy(SET CMP0043 OLD)
cmake_policy(SET CMP0048 OLD)
cmake_policy(SET CMP0057 NEW) # IF(IN_LIST)

PROJECT(OpenViBE)

# These versions are used by the subprojects by default.
# If you wish to maintain specific version numbers for a subproject, please do so in the projects CMakeLists.txt
SET(OV_GLOBAL_VERSION_MAJOR 2)
SET(OV_GLOBAL_VERSION_MINOR 2)
SET(OV_GLOBAL_VERSION_PATCH 0)
SET(OV_GLOBAL_VERSION_STRING "${OV_GLOBAL_VERSION_MAJOR}.${OV_GLOBAL_VERSION_MINOR}.${OV_GLOBAL_VERSION_PATCH}")

SET(DOCUMENTATION_TEMP_DIRECTORY "${CMAKE_INSTALL_PREFIX}/doc-tmp")

MESSAGE(STATUS "Appending compilation flags...")

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

LIST(APPEND LIST_DEPENDENCIES_PATH "${CMAKE_SOURCE_DIR}/${OV_DEPENDENCIES_PLATFORM_FOLDER_NAME}")

# Setup knowledge of GNU-style install path variables
INCLUDE("GNUInstallDirs")

if(NOT CMAKE_BUILD_TYPE AND CMAKE_GENERATOR MATCHES "Visual Studio*")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin" )
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/lib" )
	foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
		string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIGU )
		string(CONCAT DIST_ROOT ${DIST_ROOT} $<$<CONFIG:${OUTPUTCONFIGU}>:${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}>)
		set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIGU} "${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}/bin" )
		set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIGU} "${CMAKE_INSTALL_PREFIX}/${OUTPUTCONFIG}/lib" )
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
	set(DIST_SYSCONFDIR ${DIST_ROOT}/etc)
elseif(CMAKE_BUILD_TYPE AND OV_PACKAGE)
	message(FATAL_ERROR "Package function not implemented (yet) !")
elseif(CMAKE_BUILD_TYPE)
	# Add directories that OpenViBE will use to look for its components runtime, unless overridden by environment variables in the launch scripts.
	# These defines should only be used in "openvibe/ov_directories.h".
	add_definitions(-DOV_CMAKE_PATH_ROOT="${CMAKE_INSTALL_PREFIX}")
	add_definitions(-DOV_CMAKE_PATH_BIN="${CMAKE_INSTALL_FULL_BINDIR}")
	add_definitions(-DOV_CMAKE_PATH_LIB="${CMAKE_INSTALL_FULL_LIBDIR}")
	add_definitions(-DOV_CMAKE_PATH_DATA="${CMAKE_INSTALL_FULL_DATADIR}/openvibe")

	set(DIST_ROOT ${CMAKE_INSTALL_PREFIX})
	set(DIST_BINDIR ${DIST_ROOT}/bin)
	set(DIST_LIBDIR ${DIST_ROOT}/lib)
	set(DIST_INCLUDEDIR ${DIST_ROOT}/include)
	set(DIST_DATADIR ${DIST_ROOT}/share)
	set(DIST_DOCDIR ${DIST_ROOT}/doc)
	set(DIST_SYSCONFDIR ${DIST_ROOT}/etc)
else()
	message(FATAL_ERROR "Build should specify a type or use a multi-type generator (like Visual Studio)")
endif()

# Create a path usable by Unix shell scripts (used by .sh launchers)
SET(OV_PATH_DEPENCENCY_LIBS "")
FOREACH(TMP ${LIST_DEPENDENCIES_PATH})
  LIST(APPEND OV_PATH_DEPENDENCY_LIBS "${TMP}/lib")
ENDFOREACH()
string(REPLACE ";" ":" OV_PATH_PATH_DEPENDENCY_LIBS "${OV_PATH_DEPENDENCY_LIBS}")

if(CMAKE_BUILD_TYPE)
	string(TOLOWER ${CMAKE_BUILD_TYPE}"" CMAKE_BUILD_TYPE_LOWER)
	find_path(SDK_MODULE_PATH NAMES AddOpenViBESDKComponents.cmake PATHS ${LIST_DEPENDENCIES_PATH} PATH_SUFFIXES openvibe-sdk-${CMAKE_BUILD_TYPE}/share  NO_DEFAULT_PATH) # Find "regular builds
endif()
find_path(SDK_MODULE_PATH NAMES AddOpenViBESDKComponents.cmake PATHS ${OPENVIBE_SDK_PATH} PATH_SUFFIXES share NO_DEFAULT_PATH) # Finds multi-type builds and dev builds
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
if(SDK_MODULE_PATH)
	LIST(APPEND CMAKE_MODULE_PATH ${SDK_MODULE_PATH})
endif()

SET(OV_LAUNCHER_SOURCE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules/launchers")
# OpenViBE-specific helper functions that are used by the CMakeLists of the subprojects.
#INCLUDE("OvAddSingleProject")
INCLUDE("OvAddProjects")
INCLUDE("OvInstallLaunchScript")
INCLUDE("OvLinkBoostLib")
INCLUDE("OvPrint")
INCLUDE("OvMessages")
#Install early, so they can be overwritten !
set(INSTALL_SDK 1)
set(INSTALL_DESIGNER 1)
include(FindOVSDK)
include(FindOVDesigner)
# Path to the OpenViBE SDK and designer extracted packages
include(AddOpenViBESDKComponents)

ADD_DEFINITIONS("-DOV_PROJECT_NAME=\"${PROJECT_NAME}\"")
ADD_DEFINITIONS("-DOV_VERSION_MAJOR=\"${OV_GLOBAL_VERSION_MAJOR}\"")
ADD_DEFINITIONS("-DOV_VERSION_MINOR=\"${OV_GLOBAL_VERSION_MINOR}\"")
ADD_DEFINITIONS("-DOV_VERSION_PATCH=\"${OV_GLOBAL_VERSION_PATCH}\"")

SET(OV_CONFIG_SUBDIR OpenVIBE CACHE STRING "Subdirectory under user directory when configuration and logs will be saved")

ADD_DEFINITIONS("-DOV_CONFIG_SUBDIR=\"${OV_CONFIG_SUBDIR}\"")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
IF(WIN32)
	# Switch /arch:SSE2 enables vectorization. Remove if your CPU/compiler doesn't support it.
	IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
	ELSEIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
#		SET(OV_EIGEN_FLAGS "/arch:SSE2")
	ENDIF()	
	# The following is needed to keep OpenMP from hogging all the cores. Note that this env var works only for VS2013+.
	# On VS2010, it may be better to disable OpenMP.
	SET(OV_OMP_WAIT_POLICY "PASSIVE")
	# Disable /MP if you don't like VS using all the cores for compilation
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4355 /MP ${OV_EIGEN_FLAGS}")
	ADD_DEFINITIONS("-DNOMINMAX -DBOOST_ALL_NO_LIB")
	# At least Windows Vista target - if you target Windows XP (0x0501), some components may not compile
	ADD_DEFINITIONS("-D_WIN32_WINNT=0x0600")

	# Find out the compiler version, deps may use this
	SET(MSVC_SERVICE_PACK "unknown")
	# MESSAGE(STATUS "st ${CMAKE_CXX_COMPILER_VERSION}")
	IF(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "18.0.0")
		SET(MSVC_SERVICE_PACK "vc120")
	ELSEIF(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "17.0.0")
		SET(MSVC_SERVICE_PACK "vc110")
	ELSEIF(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "16.0.0")
		SET(MSVC_SERVICE_PACK "vc100")
	ENDIF()

	STRING(REGEX MATCH "vc120.*" MSVC_VER120 ${MSVC_SERVICE_PACK})
	IF(MSVC_VER120)
		# Boost 1.55 asio causes several compiler warnings on VS2013 we cannot do much about. try removing the following when boost is upgraded.
		ADD_DEFINITIONS("-D_WINSOCK_DEPRECATED_NO_WARNINGS")

		# If OpenMP is available, using it will allow Eigen to use multiple cores in matrix math. Only used on VS120,
		# as VS100 allocates the cores too aggressively when OpenMP is enabled.
		INCLUDE("FindOpenMP")
		IF(OPENMP_FOUND)
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		ENDIF()
	ENDIF()
	MESSAGE(STATUS "MSVC version is ${MSVC_SERVICE_PACK}")

	SET(OV_WIN32_BOOST_VERSION "1_58")
ELSEIF(UNIX)
	# If OpenMP is available, using it will allow Eigen to use multiple cores in matrix math.
	INCLUDE("FindOpenMP")
	IF(OPENMP_FOUND)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	ENDIF()

	# Switch -msse2 enables vectorization. Remove if your CPU/compiler doesn't support it.
#	SET(OV_EIGEN_FLAGS "-msse2")
        SET(OV_EIGEN_FLAGS "-std=c++11 -pthread")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${OV_EIGEN_FLAGS}")
	# This ENV var is needed to locate our custom IT++ on Linux
	SET(ENV{PKG_CONFIG_PATH} "${LIST_DEPENDENCIES_PATH}/lib/pkgconfig")

	if (${CMAKE_SYSTEM_PROCESSOR} MATCHES i386|i586|i686)
		set ( BIT_MODE "32")
	else ()
		set ( BIT_MODE "64")
	endif ()

	if(EXISTS "/etc/debian_version")
		set (PLATFORM "Debian")
		ADD_DEFINITIONS("-DTARGET_OS_Linux_Debian")
	endif(EXISTS "/etc/debian_version")

	if(EXISTS "/etc/fedora-release")
		set (PLATFORM "Fedora")
		ADD_DEFINITIONS("-DTARGET_OS_Linux_Fedora")
	endif(EXISTS "/etc/fedora-release")
ELSE()
	MESSAGE(WARNING "Warning: unknown platform")
ENDIF()

# IF(NOT CMAKE_BUILD_TYPE)
	# MESSAGE(STATUS "Setting default build type to Release")
	# SET(CMAKE_BUILD_TYPE "Release")
# ENDIF()

FUNCTION(SET_BUILD_PLATFORM)
	IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_x64)
		SET(PLATFORM_TARGET "x64" PARENT_SCOPE)
	ELSEIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_i386)
		SET(PLATFORM_TARGET "x86" PARENT_SCOPE)
	ELSE()
		ADD_DEFINITIONS(-DTARGET_ARCHITECTURE_Unknown)
		SET(PLATFORM_TARGET "unknown" PARENT_SCOPE)		
	ENDIF()

	IF(WIN32)
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
		ADD_DEFINITIONS(-DTARGET_OS_Windows)
		ADD_DEFINITIONS(-DTARGET_COMPILER_VisualStudio)
	ELSEIF(APPLE)
		ADD_DEFINITIONS(-fnon-call-exceptions)
		ADD_DEFINITIONS(-DTARGET_OS_MacOS)
		ADD_DEFINITIONS(-DTARGET_COMPILER_LLVM)
	ELSEIF(UNIX)
		# ADD_DEFINITIONS(-fvisibility=hidden) # This flag should be present... man gcc
		ADD_DEFINITIONS(-fnon-call-exceptions)
		ADD_DEFINITIONS(-DTARGET_OS_Linux)
		ADD_DEFINITIONS(-DTARGET_COMPILER_GCC)
	ENDIF()
ENDFUNCTION()

# In extras/ we assume that everything is built onto the same platform. 
# It is very error prone to leave this to the modules.
SET_BUILD_PLATFORM()

# Print the used compilation parameters (for transparency)
GET_DIRECTORY_PROPERTY(TMP_DEFINITIONS COMPILE_DEFINITIONS)
MESSAGE(STATUS "Compilation flags used at source root: ")
MESSAGE(STATUS "  COMPILE_DEFINITIONS = '${TMP_DEFINITIONS}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS = '${CMAKE_CXX_FLAGS}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS_RELEASE = '${CMAKE_CXX_FLAGS_RELEASE}'")
MESSAGE(STATUS "  CMAKE_CXX_FLAGS_DEBUG = '${CMAKE_CXX_FLAGS_DEBUG}'")

# if want to compile tests, use the following.
SET(OV_COMPILE_TESTS "true")

# if you don't want an Ogre dependency, use the following. All 3D visualizations and 3D apps will be unavailable. The result will compile faster.
# SET(OV_DISABLE_OGRE "true")

# if you don't want a GTK dependency, set the following. Note that most applications including Acquisition Server and Designer will NOT be built.
# This option is mostly of interest to hackers who are interested in using a minimal OpenViBE kernel in their own projects.
# SET(OV_DISABLE_GTK "true")

# By setting SKIP[_FOLDER]* you can skip a subtree (example: SKIP_A_B_C skips folder a/b/c and all its subfolders if any)

# Skip building documentation
SET(SKIP_DOCUMENTATION "1")

# Python plugin sometimes causes problems; use the following to disable it
#SET(SKIP_CONTRIB_PLUGINS_PROCESSING_PYTHON "1")

# Skipping demos can speed up compilation a lot, if you don't need them
#SET(SKIP_APPLICATIONS_DEMOS "1")
#SET(SKIP_CONTRIB_APPLICATIONS "1")

IF(UNIX)
	# On Linux, the presence of matlab plugin .so can cause crashes in Simple DSP box as the two may use different versions of boost which are
	# then wrongly exposed to the other party due to the dlopen() flag RTLD_GLOBAL. Unfortunately this flag is needed or the python plugin
	# imports no longer work... You may enable the Matlab plugin compilation and possibly set the flag to RTLD_LOCAL in ovCKernelLoader.cpp,
	# but be aware of the consequences.
	MESSAGE(STATUS "Note: Disabling Matlab plugin compilation by default")
	SET(SKIP_PLUGINS_PROCESSING_MATLAB "1")
ENDIF()

#SET(SKIP_MODULES_AUTOMATON "1")
#SET(SKIP_MODULES_EBML "1")
#SET(SKIP_MODULES_FS "1")
#SET(SKIP_MODULES_SOCKET "1")
#SET(SKIP_MODULES_STREAM "1")
#SET(SKIP_MODULES_SYSTEM "1")
#SET(SKIP_MODULES_XML "1")
#SET(SKIP_APPLICATIONS_PLATFORM_ACQUISITION-SERVER "1")
#SET(SKIP_PLUGINS_PROCESSING_EXAMPLES "1")

# FIXME CERT > From OpenViBE designer repo in the future
# Designer is currently in another repository; taken from there
SET(SKIP_APPLICATIONS_PLATFORM_DESIGNER "0")
SET(SKIP_APPLICATIONS_EXAMPLES_CONVERT "0")		# Convert depends on Designer

# If you wish you can uncomment the next line and add a file with additional information called OVBranches.cmake to
# the cmake-modules folder. It makes modifying this very CMake file easier.
#INCLUDE("OVCustomSettings")

# a list of all project which will be filled by the directory traversal. This is needed to generate the documentation.
SET_PROPERTY(GLOBAL PROPERTY OV_PROP_CURRENT_PROJECTS "")
SET_PROPERTY(GLOBAL PROPERTY OV_PROP_CURRENT_PROJECTS_BUILD_DIR "")

# Used by the various Find* scripts to locate OpenViBE modules
# Used by the various Find* scripts to locate OpenViBE modules
SET(OV_BASE_DIR ${CMAKE_SOURCE_DIR})

# needed for making visual studio projects when this script is called without CMAKE_BUILD_TYPE
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG "TARGET_BUILDTYPE_Debug")
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE "TARGET_BUILDTYPE_Release")
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELWITHDEBINFO "TARGET_BUILDTYPE_Release")

SET(CMAKE_INSTALL_FULL_DOCDIR "${CMAKE_INSTALL_FULL_BINDIR}/../doc/html")

ADD_SUBDIRECTORY("build-tool/")
# Traverse these directories and build their components
ADD_SUBDIRECTORY("modules/tcptagging")
ADD_SUBDIRECTORY("plugins/")
ADD_SUBDIRECTORY("applications/")
ADD_SUBDIRECTORY("externals/")
ADD_SUBDIRECTORY("contrib/")
ADD_SUBDIRECTORY("documentation/") # needs to be the last since it uses the list of collected projects

#add dependency to plugin inspector to the documentation
IF(openvibe-documentation)
	ADD_DEPENDENCIES(openvibe-documentation openvibe-plugin-inspector)
ENDIF(openvibe-documentation)

# make a copy of the dependency script, this is done so we can use the same launch scripts both in win install and win devel situations.
IF(WIN32)
	FIND_PATH(OV_DEPENDENCY_CMD_PATH windows-dependencies.cmd PATHS ${LIST_DEPENDENCIES_PATH} NO_DEFAULT_PATH)
	IF(NOT OV_DEPENDENCY_CMD_PATH)
		MESSAGE(FATAL_ERROR "Dependency installer should have generated 'windows-dependencies.cmd'. Looked from ${LIST_DEPENDENCIES_PATH}.")
	ENDIF()
	INSTALL(PROGRAMS ${OV_DEPENDENCY_CMD_PATH}/windows-dependencies.cmd DESTINATION ${DIST_BINDIR} RENAME "openvibe-set-env.cmd")
ENDIF(WIN32)


# ----------------------
# Generate 'external application launcher' script
# ----------------------
IF(WIN32)
	SET(OV_EXT_CMD_FILE "openvibe-external-application-launcher.cmd")
ELSEIF(UNIX)
	SET(OV_EXT_CMD_FILE "openvibe-external-application-launcher")
ENDIF()

CONFIGURE_FILE("${OV_LAUNCHER_SOURCE_PATH}/${OV_EXT_CMD_FILE}-base" "${CMAKE_CURRENT_BINARY_DIR}/${OV_EXT_CMD_FILE}")
INSTALL(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${OV_EXT_CMD_FILE} DESTINATION ${DIST_BINDIR})

# create the openvibe-ogre-plugins.cfg file for Ogre on UNIX
IF(UNIX)
	execute_process(COMMAND pkg-config --variable=plugindir OGRE OUTPUT_VARIABLE OGRE3D_PLUGIN_PATH)
	CONFIGURE_FILE("${OV_LAUNCHER_SOURCE_PATH}/openvibe-ogre-plugins.cfg-base" "${CMAKE_CURRENT_BINARY_DIR}/openvibe-ogre-plugins.cfg")
	INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/openvibe-ogre-plugins.cfg DESTINATION ${DIST_DATADIR}/openvibe)
ENDIF()

SET(INCLUDED_OV_SDK_COMPONENTS DEPENDENCIES ALLPLUGINS)
INCLUDE("AddOpenViBESDKComponents")

configure_file(test/CTestTestfile.cmake . @ONLY)
