# Building tested with yagarto that uses
#   binutils: 2.21
#   gcc:      4.6.2
#   newlib:   1.19.0
#   gdb:      7.3.1
#   (19.11.2011, mifi)

# Name of target
TARGET = sumo

# NXTOSEK root path
NXTOSEKROOT = external/nxtosek

# Paths usually set up in NXTOSEKROOT\ecrobot\tools_gcc.mak
GNUARM_ROOT = C:/yagarto
NEXTTOOL_ROOT = C:/Users/smiler/Documents/Development/nxt_sumo/external

########################################################################

TARGET_SOURCES := \
        $(TARGET).c
TOPPERS_OSEK_OIL_SOURCE := ./$(TARGET).oil

O_PATH ?= build

include $(NXTOSEKROOT)/ecrobot/ecrobot.mak