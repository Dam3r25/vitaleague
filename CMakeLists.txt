cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Define la variable de entorno VITASDK.")
  endif()
endif()

project(vitaleague C)
include("$ENV{VITASDK}/share/vita.cmake" REQUIRED)

set(VITA_APP_NAME "Vita League RL")
set(VITA_TITLEID  "VLEA00001")
set(VITA_VERSION  "04.00")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-q -Wall -O3 -std=gnu99 -ftree-vectorize")

add_executable(${PROJECT_NAME} main.c)

target_link_libraries(${PROJECT_NAME}
  vitaGL
  stdc++
  SceLibKernel_stub
  SceAppMgr_stub
  SceAppUtil_stub
  mathneon
  c
  SceCommonDialog_stub
  m
  SceGxm_stub
  SceDisplay_stub
  SceSysmodule_stub
  z
  vitashark
  SceShaccCgExt
  taihen_stub
  SceShaccCg_stub
  SceKernelDmacMgr_stub
  SceCtrl_stub
)

vita_create_self(${PROJECT_NAME}.self ${PROJECT_NAME})
vita_create_vpk(${PROJECT_NAME}.vpk ${VITA_TITLEID} ${PROJECT_NAME}.self
  VERSION ${VITA_VERSION}
  NAME ${VITA_APP_NAME}
)
