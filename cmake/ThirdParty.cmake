include_guard(GLOBAL)

get_filename_component(RM_CONTROL_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(RM_THIRD_PARTY_ROOT "${RM_CONTROL_ROOT}/third_party")

set(MUJOCO_ROOT "" CACHE PATH "Optional MuJoCo SDK root. Overrides third_party/mujoco when set.")

function(_rm_append_existing_candidate out_var candidate)

    if(candidate)
        get_filename_component(_candidate "${candidate}" ABSOLUTE)
        set(_items ${${out_var}})
        list(APPEND _items "${_candidate}")
        set(${out_var} "${_items}" PARENT_SCOPE)
    endif()
endfunction()

function(rm_find_mujoco)
    if(TARGET third_party::mujoco)
        return()
    endif()

    # 查找候选路径列表
    set(_candidates)
    _rm_append_existing_candidate(_candidates "${MUJOCO_ROOT}")
    _rm_append_existing_candidate(_candidates "${RM_THIRD_PARTY_ROOT}/mujoco")
    _rm_append_existing_candidate(_candidates "$ENV{MUJOCO_ROOT}")
    _rm_append_existing_candidate(_candidates "${RM_CONTROL_ROOT}/../mujoco-3.3.0")
    _rm_append_existing_candidate(_candidates "${RM_CONTROL_ROOT}/../mujoco")

    list(REMOVE_DUPLICATES _candidates)

    foreach(_candidate IN LISTS _candidates)
        if(EXISTS "${_candidate}/include/mujoco/mujoco.h")
            find_library(_mujoco_library
                NAMES mujoco
                HINTS "${_candidate}/lib" "${_candidate}/bin"
                NO_DEFAULT_PATH
            )
            if(_mujoco_library)
                set(_mujoco_root "${_candidate}")
                break()
            endif()
        endif()
    endforeach()

    if(NOT _mujoco_root)
        string(REPLACE ";" "\n  " _checked "${_candidates}")
        message(FATAL_ERROR
            "MuJoCo SDK not found.\n"
            "Put it under:\n"
            "  ${RM_THIRD_PARTY_ROOT}/mujoco\n"
            "Expected layout:\n"
            "  third_party/mujoco/include/mujoco/mujoco.h\n"
            "  third_party/mujoco/lib/libmujoco.so or libmujoco.so.<version>\n"
            "Or configure with:\n"
            "  cmake -DMUJOCO_ROOT=/path/to/mujoco ...\n"
            "Checked:\n"
            "  ${_checked}"
        )
    endif()

    get_filename_component(_mujoco_library_dir "${_mujoco_library}" DIRECTORY)

    add_library(third_party_mujoco UNKNOWN IMPORTED GLOBAL)
    set_target_properties(third_party_mujoco PROPERTIES
        IMPORTED_LOCATION "${_mujoco_library}"
        INTERFACE_INCLUDE_DIRECTORIES "${_mujoco_root}/include;${RM_THIRD_PARTY_ROOT}/include"
    )
    add_library(third_party::mujoco ALIAS third_party_mujoco)

    set(RM_MUJOCO_ROOT "${_mujoco_root}" CACHE INTERNAL "Resolved MuJoCo SDK root")
    set(RM_MUJOCO_LIBRARY "${_mujoco_library}" CACHE INTERNAL "Resolved MuJoCo library")
    set(RM_MUJOCO_LIBRARY_DIR "${_mujoco_library_dir}" CACHE INTERNAL "Resolved MuJoCo library directory")

    message(STATUS "Using MuJoCo SDK: ${_mujoco_root}")
endfunction()

function(rm_target_link_mujoco target)
    rm_find_mujoco()
    target_link_libraries("${target}" PRIVATE third_party::mujoco)
    set_property(TARGET "${target}" APPEND PROPERTY BUILD_RPATH "${RM_MUJOCO_LIBRARY_DIR}")
endfunction()

function(rm_target_link_glfw target)
    if(EXISTS "${RM_THIRD_PARTY_ROOT}/glfw/CMakeLists.txt")
        if(NOT TARGET glfw)
            set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
            set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
            set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
            add_subdirectory(
                "${RM_THIRD_PARTY_ROOT}/glfw"
                "${CMAKE_BINARY_DIR}/third_party/glfw"
                EXCLUDE_FROM_ALL
            )
        endif()
        target_include_directories("${target}" PRIVATE "${RM_THIRD_PARTY_ROOT}/include")
        target_link_libraries("${target}" PRIVATE glfw)
    else()
        find_package(glfw3 REQUIRED)
        target_include_directories("${target}" PRIVATE "${RM_THIRD_PARTY_ROOT}/include")
        target_link_libraries("${target}" PRIVATE glfw)
    endif()
endfunction()
