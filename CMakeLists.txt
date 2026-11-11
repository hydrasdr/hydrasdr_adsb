# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(hydrasdr_adsb VERSION 0.5.1 LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# vcpkg's MSBuild integration would list the copied libhydrasdr DLLs as its
# own outputs and delete them on a clean or rebuild.
if(MSVC)
	set(CMAKE_VS_GLOBALS "VcpkgEnabled=false" "VcpkgApplocalDeps=false")
endif()

if(MSVC)
	add_compile_options(/W4 /WX)
	add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
	add_compile_options(-Wall -Wextra -Wshadow -Wconversion -Werror
		-fno-math-errno)
endif()

add_library(adsb_core STATIC
	src/cpr.c
	src/crc.c
	src/demod.c
	src/icao.c
	src/input_file.c
	src/iq_record.c
	src/modes.c
	src/output.c
	src/raw12.c
	src/ring.c
	src/track.c)
target_include_directories(adsb_core PUBLIC src)
if(NOT MSVC)
	target_link_libraries(adsb_core PUBLIC m)
endif()
if(NOT WIN32)
	find_package(Threads REQUIRED)
	target_link_libraries(adsb_core PUBLIC Threads::Threads)
endif()

add_executable(hydrasdr_adsb src/main.c src/net.c)
target_link_libraries(hydrasdr_adsb PRIVATE adsb_core)
if(WIN32)
	target_link_libraries(hydrasdr_adsb PRIVATE ws2_32)
endif()
target_compile_definitions(hydrasdr_adsb PRIVATE
	HYDRASDR_ADSB_VERSION="${PROJECT_VERSION}")
if(MINGW)
	target_link_options(hydrasdr_adsb PRIVATE -static)
endif()

# Live input through libhydrasdr 1.1.3 or later, found by pkg-config or
# given as HYDRASDR_INCLUDE_DIR (holding hydrasdr.h) and HYDRASDR_LIBRARY.
# On Windows, when they are not given, the latest stable hydrasdr-host
# release is downloaded. Without libhydrasdr the program reads recordings
# only.
option(WITH_LIBHYDRASDR "live input through libhydrasdr" ON)
option(HYDRASDR_DOWNLOAD "Windows: download the latest stable libhydrasdr" ON)
set(HYDRASDR_INCLUDE_DIR "" CACHE PATH "directory holding hydrasdr.h")
set(HYDRASDR_LIBRARY "" CACHE FILEPATH "libhydrasdr or its import library")
# HYDRASDR_RUNTIME and HYDRASDR_LICENSE are set only by the download below:
# an INTERNAL cache entry is always forced, so an initial empty set here
# would clear them at every later configure

# The Windows x64 zip of the release, checked against the SHA-256 GitHub
# lists for it, and the headers from the source archive of the same tag.
# The URLs are built from the repository and the tag, never taken from the
# reply.
macro(hydrasdr_dl_check what)
	list(GET st 0 err)
	if(err)
		message(FATAL_ERROR "download, ${what}: ${st}")
	endif()
endmacro()
if(WIN32 AND WITH_LIBHYDRASDR AND HYDRASDR_DOWNLOAD AND
   NOT (HYDRASDR_INCLUDE_DIR AND HYDRASDR_LIBRARY))
	if(CMAKE_VERSION VERSION_LESS 3.19)
		message(FATAL_ERROR "the libhydrasdr download needs CMake 3.19")
	endif()
	set(repo "hydrasdr/hydrasdr-host")
	set(dl "${CMAKE_BINARY_DIR}/libhydrasdr")
	file(REMOVE_RECURSE "${dl}")
	file(DOWNLOAD "https://api.github.com/repos/${repo}/releases/latest"
		"${dl}/latest.json" TLS_VERIFY ON STATUS st
		HTTPHEADER "Accept: application/vnd.github+json")
	hydrasdr_dl_check("libhydrasdr latest release")
	file(READ "${dl}/latest.json" rel)
	string(JSON tag GET "${rel}" tag_name)
	if(NOT tag MATCHES "^v([0-9]+\\.[0-9]+\\.[0-9]+)$")
		message(FATAL_ERROR "libhydrasdr release tag '${tag}' not understood")
	endif()
	set(ver "${CMAKE_MATCH_1}")
	if(ver VERSION_LESS 1.1.3)
		message(FATAL_ERROR "libhydrasdr ${ver} is older than 1.1.3")
	endif()
	set(zip "hydrasdr-host-tools-Windows-x64-${tag}.zip")
	set(sha "")
	string(JSON n LENGTH "${rel}" assets)
	foreach(i RANGE ${n})
		if(i LESS n)
			string(JSON name GET "${rel}" assets ${i} name)
			if(name STREQUAL zip)
				string(JSON sha GET "${rel}" assets ${i} digest)
			endif()
		endif()
	endforeach()
	string(REGEX REPLACE "^sha256:" "" sha "${sha}")
	string(LENGTH "${sha}" len)
	if(NOT len EQUAL 64 OR NOT sha MATCHES "^[0-9a-f]+$")
		message(FATAL_ERROR "libhydrasdr ${tag}: no SHA-256 for ${zip}")
	endif()
	file(DOWNLOAD "https://github.com/${repo}/releases/download/${tag}/${zip}"
		"${dl}/${zip}" TLS_VERIFY ON EXPECTED_HASH SHA256=${sha}
		STATUS st)
	hydrasdr_dl_check("${zip}")
	file(DOWNLOAD "https://github.com/${repo}/archive/refs/tags/${tag}.tar.gz"
		"${dl}/source.tar.gz" TLS_VERIFY ON STATUS st)
	hydrasdr_dl_check("libhydrasdr source archive")
	file(ARCHIVE_EXTRACT INPUT "${dl}/${zip}" DESTINATION "${dl}/win")
	file(ARCHIVE_EXTRACT INPUT "${dl}/source.tar.gz" DESTINATION "${dl}/src"
		PATTERNS "*/libhydrasdr/src/*.h" "*/libhydrasdr/LICENSE.md")
	file(GLOB_RECURSE lib "${dl}/win/hydrasdr.lib")
	file(GLOB_RECURSE dlls "${dl}/win/*.dll")
	file(GLOB_RECURSE hdr "${dl}/src/hydrasdr.h")
	list(LENGTH lib nlib)
	list(LENGTH hdr nhdr)
	if(NOT nlib EQUAL 1 OR NOT nhdr EQUAL 1 OR NOT dlls)
		message(FATAL_ERROR "libhydrasdr ${tag}: unexpected archive layout")
	endif()
	get_filename_component(inc "${hdr}" DIRECTORY)
	set(HYDRASDR_INCLUDE_DIR "${inc}" CACHE PATH
		"directory holding hydrasdr.h" FORCE)
	set(HYDRASDR_LIBRARY "${lib}" CACHE FILEPATH
		"libhydrasdr or its import library" FORCE)
	set(HYDRASDR_RUNTIME "${dlls}" CACHE INTERNAL
		"DLLs copied next to the program")
	file(GLOB_RECURSE lic "${dl}/src/*/libhydrasdr/LICENSE.md")
	set(HYDRASDR_LICENSE "${lic}" CACHE INTERNAL
		"license of the downloaded libhydrasdr, for the package")
	message(STATUS "libhydrasdr ${ver} downloaded, ${zip} SHA-256 ${sha}")
endif()

set(hydrasdr_target "")
if(WITH_LIBHYDRASDR AND HYDRASDR_INCLUDE_DIR AND HYDRASDR_LIBRARY)
	add_library(libhydrasdr UNKNOWN IMPORTED)
	set_target_properties(libhydrasdr PROPERTIES
		IMPORTED_LOCATION "${HYDRASDR_LIBRARY}"
		INTERFACE_INCLUDE_DIRECTORIES "${HYDRASDR_INCLUDE_DIR}")
	set(hydrasdr_target libhydrasdr)
elseif(WITH_LIBHYDRASDR)
	find_package(PkgConfig QUIET)
	if(PKG_CONFIG_FOUND)
		pkg_check_modules(LIBHYDRASDR QUIET IMPORTED_TARGET
			libhydrasdr>=1.1.3)
		if(LIBHYDRASDR_FOUND)
			set(hydrasdr_target PkgConfig::LIBHYDRASDR)
		endif()
	endif()
endif()
if(hydrasdr_target)
	target_sources(hydrasdr_adsb PRIVATE src/input_hydrasdr.c)
	target_link_libraries(hydrasdr_adsb PRIVATE ${hydrasdr_target})
	target_compile_definitions(hydrasdr_adsb PRIVATE HAVE_LIBHYDRASDR)
	if(HYDRASDR_RUNTIME)
		add_custom_command(TARGET hydrasdr_adsb POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${HYDRASDR_RUNTIME} "$<TARGET_FILE_DIR:hydrasdr_adsb>")
	endif()
	message(STATUS "libhydrasdr: live input")
else()
	message(STATUS "libhydrasdr not found: recordings only")
endif()

# Tools and tests that are not part of the release
include("${CMAKE_CURRENT_SOURCE_DIR}/internal.cmake" OPTIONAL)

# Windows zip, built by "cmake --build . --config Release --target package":
# the program, hydrasdr.dll of the downloaded libhydrasdr release, the
# libusb-1.0.dll of the MinGW64 build in the libusb release archive, and the
# license texts. That libusb-1.0.dll imports only Windows system DLLs; the
# Visual Studio builds import vcruntime140.dll. The libusb archive and its
# license text (LGPL-2.1) are downloaded once, checked by SHA-256.
set(LIBUSB_COPYING_URL
	"https://raw.githubusercontent.com/libusb/libusb/v1.0.30/COPYING")
set(LIBUSB_COPYING_SHA256
	"5df07007198989c622f5d41de8d703e7bef3d0e79d62e24332ee739a452af62a")
set(LIBUSB_7Z_URL
	"https://github.com/libusb/libusb/releases/download/v1.0.30/libusb-1.0.30.7z")
set(LIBUSB_7Z_SHA256
	"7fb1dfec805b97983763d7d0ae244320da12add1003d4249c96cc4d586398c79")
# out: kept in the build directory, downloaded when missing or wrong
function(libusb_fetch url sha out)
	set(h "")
	if(EXISTS "${out}")
		file(SHA256 "${out}" h)
	endif()
	if(NOT h STREQUAL sha)
		if(NOT HYDRASDR_DOWNLOAD)
			message(FATAL_ERROR "package: ${out} missing or wrong SHA-256")
		endif()
		file(DOWNLOAD "${url}" "${out}" TLS_VERIFY ON
			EXPECTED_HASH SHA256=${sha} STATUS st)
		hydrasdr_dl_check("${url}")
	endif()
endfunction()
if(MSVC AND HYDRASDR_RUNTIME AND HYDRASDR_LICENSE)
	set(usb_lic "${CMAKE_BINARY_DIR}/libusb-1.0-COPYING.txt")
	set(usb_7z "${CMAKE_BINARY_DIR}/libusb-1.0.30.7z")
	set(usb_dir "${CMAKE_BINARY_DIR}/libusb-1.0.30")
	set(usb_dll "${usb_dir}/MinGW64/dll/libusb-1.0.dll")
	libusb_fetch("${LIBUSB_COPYING_URL}" ${LIBUSB_COPYING_SHA256} "${usb_lic}")
	libusb_fetch("${LIBUSB_7Z_URL}" ${LIBUSB_7Z_SHA256} "${usb_7z}")
	file(REMOVE_RECURSE "${usb_dir}")
	file(ARCHIVE_EXTRACT INPUT "${usb_7z}" DESTINATION "${usb_dir}"
		PATTERNS "MinGW64/dll/libusb-1.0.dll")
	if(NOT EXISTS "${usb_dll}")
		message(FATAL_ERROR "package: no MinGW64/dll/libusb-1.0.dll in ${usb_7z}")
	endif()
	set(rt "")
	foreach(f ${HYDRASDR_RUNTIME})
		get_filename_component(n "${f}" NAME)
		if(NOT n STREQUAL "libusb-1.0.dll")
			list(APPEND rt "${f}")
		endif()
	endforeach()
	install(TARGETS hydrasdr_adsb RUNTIME DESTINATION .)
	install(FILES ${rt} "${usb_dll}" README.md LICENSE DESTINATION .)
	install(FILES "${HYDRASDR_LICENSE}" DESTINATION licenses
		RENAME libhydrasdr-LICENSE.md)
	install(FILES "${usb_lic}" DESTINATION licenses)
	set(CPACK_GENERATOR ZIP)
	set(CPACK_PACKAGE_FILE_NAME
		"hydrasdr_adsb-${PROJECT_VERSION}-windows-x64")
	set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
	message(STATUS "package: ${CPACK_PACKAGE_FILE_NAME}.zip, ${usb_dll}")
	include(CPack)
endif()
