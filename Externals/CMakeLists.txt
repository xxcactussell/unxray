if (XRAY_USE_LUAJIT)
    add_subdirectory(LuaJIT-proj)
else()
    find_package(Lua51 REQUIRED)
endif()

add_subdirectory(luabind)
add_subdirectory(xrLuaFix)

set(ALSOFT_UTILS OFF CACHE BOOL "Disable openal-soft utils" FORCE)
set(ALSOFT_EXAMPLES OFF CACHE BOOL "Disable openal-soft examples" FORCE)
set(ALSOFT_TESTS OFF CACHE BOOL "Disable openal-soft tests" FORCE)
add_subdirectory(openal-soft)

# Silence all warnings for external libraries
add_compile_options(${XRAY_DISABLE_WARNINGS})

add_subdirectory(GameSpy)
add_subdirectory(imgui-proj)

if (NOT TARGET xrLuabind)
    message(FATAL_ERROR
        "You probably have downloaded OpenXRay sources as ZIP archive, you can't do that. Use git to clone the repository.\n"
        "Read the build instructions: https://github.com/OpenXRay/xray-16/wiki"
    )
endif()
