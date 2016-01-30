
#serial UDP

ifeq ($(TARGET), ap)
include $(CFG_SHARED)/telemetry_transparent_udp.makefile
endif

ap.srcs += s $(SRC_FIRMWARE)/rotorcraft_telemetry.c
