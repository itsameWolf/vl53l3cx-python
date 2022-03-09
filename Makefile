CC = $(CROSS_COMPILE)gcc
RM = rm

#CFLAGS = -O0 -g -Wall -c
CFLAGS = -O2 -Wall -c -fPIC

OUTPUT_DIR = bin
OBJ_DIR = obj

ROOT_DIR := $(shell pwd)
API_DIR := $(ROOT_DIR)/Api

TARGET_LIB = $(OUTPUT_DIR)/vl53lx_python

INCLUDES = \
	-I$(ROOT_DIR) \
	-I$(API_DIR)/core/inc \
	-I$(ROOT_DIR)/platform/inc

PYTHON_INCLUDES = \
    -I/usr/include/python3.8

VPATH = \
	$(API_DIR)/core/src \
	$(ROOT_DIR)/platform/src/ \
	$(ROOT_DIR)/python_lib

LIB_SRCS = \
	vl53lx_api.c \
	vl53lx_api_calibration.c \
	vl53lx_api_core.c \
	vl53lx_api_debug.c \
	vl53lx_api_preset_modes.c \
	vl53lx_core.c \
	vl53lx_core_support.c \
	vl53lx_dmax.c \
	vl53lx_hist_algos_gen3.c \
	vl53lx_hist_algos_gen4.c \
	vl53lx_hist_char.c \
	vl53lx_hist_core.c \
	vl53lx_hist_funcs.c \
	vl53lx_nvm.c \
	vl53lx_nvm_debug.c \
	vl53lx_register_funcs.c \
	vl53lx_sigma_estimate.c \
	vl53lx_silicon_core.c \
	vl53lx_wait.c \
	vl53lx_xtalk.c \
  	vl53lx_platform.c \
  	vl53lx_platform_ipp.c\
  	vl53lx_python.c

LIB_OBJS  = $(LIB_SRCS:%.c=$(OBJ_DIR)/%.o)

.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(LIB_OBJS)
	mkdir -p $(dir $@)
	$(CC) -shared $^ $(PYTHON_INCLUDES) $(INCLUDES) -lpthread -o $@.so

$(OBJ_DIR)/%.o:%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(PYTHON_INCLUDES) $(INCLUDES) $< -o $@

.PHONY: clean
clean:
	-${RM} -rf ./$(OUTPUT_DIR) ./$(OBJ_DIR)

