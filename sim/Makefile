# Host flight simulator for the CanSat firmware.
#
# Links the real src/*.c (minus the SD-card driver and the interrupt vector
# file, which need hardware) against the stub HAL and the device fakes used by
# the unit tests.  src/main.c is not modified and still owns main(); sim/world.c
# installs a constructor that primes the devices and a tick hook that drives
# the flight profile, so the firmware's own superloop is what runs.
#
#   make            build sim/build/firmware-host
#   make run        build, then fly (writes sim/out/sd.csv and lora.jsonl)
#   make clean

CC      ?= clang
PYTHON  ?= python3
ROOT    := $(abspath ..)
OUT     := $(abspath out)
BUILD   := build

CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter \
           -Wno-unused-function -I$(ROOT)/src -I$(ROOT)/test/fakes \
           -I. -DSIM_OUT_DIR=\"$(OUT)\"
LDLIBS  := -lm

SRC      := $(wildcard $(ROOT)/src/*.c)
# Needs FatFS / the vector table: replaced on the host.
SKIP     := $(ROOT)/src/sdlog.c $(ROOT)/src/stm32g4xx_it.c
FIRMWARE := $(filter-out $(SKIP),$(SRC))
HARNESS  := world.c fake_sdlog.c
FAKES    := $(ROOT)/test/fakes/fake_hal.c $(ROOT)/test/fakes/fake_devices.c

OBJS     := $(patsubst $(ROOT)/%.c,$(BUILD)/%.c.o,$(FIRMWARE) $(FAKES)) \
            $(patsubst %.c,$(BUILD)/%.c.o,$(HARNESS))

all: $(BUILD)/firmware-host

$(BUILD)/firmware-host: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDLIBS)

$(BUILD)/%.c.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

run: $(BUILD)/firmware-host
	@mkdir -p $(OUT)
	$(BUILD)/firmware-host

# Fly, then load the artifacts through analysis/cansat_data.py.
check: run
	$(PYTHON) check_roundtrip.py

clean:
	rm -rf $(BUILD) $(OUT)

.PHONY: all run check clean
