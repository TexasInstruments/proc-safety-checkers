# File: safety_checkers_component.mk
#       This file is component include for safety checkers.
# List of variables set in this file and their purpose:
# <mod>_RELPATH        - This is the relative path of the module, typically from
#                        top-level directory of the package
# <mod>_PATH           - This is the absolute path of the module. It derives from
#                        absolute path of the top-level directory (set in env.mk)
#                        and relative path set above
# <mod>_INCLUDE        - This is the path that has interface header files of the
#                        module. This can be multiple directories (space separated)
# <mod>_PKG_LIST       - Names of the modules (and sub-modules) that are a part
#                        part of this module, including itself.
# <mod>_BOARD_DEPENDENCY - "yes": means the code for this module depends on
#                             platform and the compiled obj/lib has to be kept
#                             under <platform> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no platform dependent code and hence
#                             the obj/libs are not kept under <platform> dir.
# <mod>_CORE_DEPENDENCY     - "yes": means the code for this module depends on
#                             core and the compiled obj/lib has to be kept
#                             under <core> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no core dependent code and hence
#                             the obj/libs are not kept under <core> dir.
# <mod>_APP_STAGE_FILES     - List of source files that belongs to the module
#                             <mod>, but that needs to be compiled at application
#                             build stage (in the context of the app). This is
#                             primarily for link time configurations or if the
#                             source file is dependent on options/defines that are
#                             application dependent. This can be left blank or
#                             not defined at all, in which case, it means there
#                             no source files in the module <mod> that are required
#                             to be compiled in the application build stage.
#
ifeq ($(safety_checkers_component_make_include), )

safety_checkers_SOCLIST         = j7200 j721e j721s2 j784s4
safety_checkers_BOARDLIST       = j7200_evm j721e_evm j721s2_evm j784s4_evm
safety_checkers_j7200_CORELIST  = mcu1_0 mcu1_1 mcu2_0
safety_checkers_j721e_CORELIST  = mcu1_0 mcu1_1 mcu2_0
safety_checkers_j721s2_CORELIST = mcu1_0 mcu1_1 mcu2_0
safety_checkers_j784s4_CORELIST = mcu1_0 mcu1_1 mcu2_0
safety_checkers_RTOS_LIST       = $(DEFAULT_RTOS_LIST)

############################
# Safety checkers package
# List of components included under Safety checkers lib
# The components included here are built and will be part of Safety checkers lib
############################
safety_checkers_LIB_LIST =

############################
# Safety checkers examples
# List of examples under Safety checkers (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
safety_checkers_EXAMPLE_LIST =

# RM PM safety checker library
safety_checkers_rm_pm_COMP_LIST = safety_checkers_rm_pm
safety_checkers_rm_pm_RELPATH = ti/safety_checkers/src
safety_checkers_rm_pm_PATH = $(SAFETY_CHECKERS_COMP_PATH)/src
safety_checkers_rm_pm_LIBNAME = safety_checkers_rm_pm
safety_checkers_rm_pm_LIBPATH = $(SAFETY_CHECKERS_COMP_PATH)/lib
safety_checkers_rm_pm_MAKEFILE = -fmakefile
export safety_checkers_rm_pm_MAKEFILE
export safety_checkers_rm_pm_LIBNAME
export safety_checkers_rm_pm_LIBPATH
safety_checkers_rm_pm_BOARD_DEPENDENCY = no
safety_checkers_rm_pm_CORE_DEPENDENCY = yes
export safety_checkers_rm_pm_COMP_LIST
export safety_checkers_rm_pm_BOARD_DEPENDENCY
export safety_checkers_rm_pm_CORE_DEPENDENCY
safety_checkers_rm_pm_PKG_LIST = safety_checkers_rm_pm
safety_checkers_rm_pm_INCLUDE = $(safety_checkers_rm_pm_PATH)
safety_checkers_rm_pm_SOCLIST = $(safety_checkers_SOCLIST)
export safety_checkers_rm_pm_SOCLIST
safety_checkers_rm_pm_$(SOC)_CORELIST = $(safety_checkers_$(SOC)_CORELIST)
export safety_checkers_rm_pm_$(SOC)_CORELIST
safety_checkers_LIB_LIST += safety_checkers_rm_pm

export safety_checkers_LIB_LIST

safety_checkers_component_make_include := 1
endif