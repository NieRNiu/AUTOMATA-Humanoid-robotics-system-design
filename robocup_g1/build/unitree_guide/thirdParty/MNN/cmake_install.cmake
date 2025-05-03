# Install script for directory: /home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nier/robocup_g1/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/MNN" TYPE FILE FILES
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/MNNDefine.h"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/Interpreter.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/HalideRuntime.h"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/Tensor.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/ErrorCode.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/ImageProcess.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/Matrix.h"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/Rect.h"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/MNNForwardType.h"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/AutoTime.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/MNNSharedContext.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/MNN/expr" TYPE FILE FILES
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/Expr.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/ExprCreator.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/MathOp.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/NeuralNetWorkOp.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/Optimizer.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/Executor.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/Module.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/NeuralNetWorkOp.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/ExecutorScope.hpp"
    "/home/nier/robocup_g1/src/unitree_guide/thirdParty/MNN/include/MNN/expr/Scope.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/nier/robocup_g1/devel/lib/libMNN.a")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nier/robocup_g1/build/unitree_guide/thirdParty/MNN/express/cmake_install.cmake")
  include("/home/nier/robocup_g1/build/unitree_guide/thirdParty/MNN/tools/converter/cmake_install.cmake")
  include("/home/nier/robocup_g1/build/unitree_guide/thirdParty/MNN/tools/cv/cmake_install.cmake")

endif()

