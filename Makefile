# Project Name
TARGET = CrossoverSerial

# Sources
CPP_SOURCES = CrossoverSerial.cpp

# Library Locations
LIBDAISY_DIR = ../../DaisyExamples/libDaisy/
DAISYSP_DIR = ../../DaisyExamples/DaisySP/

# Add CMSIS sources and headers for f32 biquad cascade
C_SOURCES = \
$(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c \
$(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c

C_INCLUDES = \
-I$(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Include \
-I./


# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
