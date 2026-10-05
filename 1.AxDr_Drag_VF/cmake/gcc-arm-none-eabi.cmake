# =============================================================================
# arm-none-eabi-gcc 交叉编译工具链文件 (给 CLion 用)
#
# CLion 用法:
#   Settings > Build, Execution, Deployment > Toolchains
#     或者直接在 CMake 配置里指定:
#       -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
#
# 工具链路径默认用 CLion 自带的那份 (D:/CLion/arm-gnu-toolchain-15.3),
# 想换别的版本时用 -DARM_TOOLCHAIN_DIR=<路径> 覆盖即可。
# =============================================================================

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT DEFINED ARM_TOOLCHAIN_DIR)
    set(ARM_TOOLCHAIN_DIR "D:/CLion/arm-gnu-toolchain-15.3")
endif()

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}g++.exe")

set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}objcopy.exe" CACHE FILEPATH "objcopy")
set(CMAKE_OBJDUMP "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}objdump.exe" CACHE FILEPATH "objdump")
set(CMAKE_SIZE    "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}size.exe"    CACHE FILEPATH "size")
set(CMAKE_AR      "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}ar.exe"      CACHE FILEPATH "ar")
set(CMAKE_RANLIB  "${ARM_TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PREFIX}ranlib.exe"  CACHE FILEPATH "ranlib")

# 裸机目标, 让 CMake 跳过"能不能链接成可执行文件"的探测
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 只在工具链目录里找库/头文件, 不要用宿主机的
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
