# ── Toolchain ─────────────────────────────────────────────────
# Override on the command line if yours differs, e.g.:
#   make CROSS=riscv64-unknown-elf-
CROSS  ?= riscv32-unknown-elf-
CC      = $(CROSS)gcc
OBJDUMP = $(CROSS)objdump
OBJCOPY = $(CROSS)objcopy

# ── Flags ─────────────────────────────────────────────────────
ARCH   = -march=rv32im_zicsr -mabi=ilp32
CFLAGS = $(ARCH) -O2 -g -Wall -nostdlib -ffreestanding
AFLAGS = $(ARCH) -g
LDFLAGS= $(ARCH) -nostdlib -T link.ld -lgcc

# ── Sources ───────────────────────────────────────────────────
ASM_SRC = start.S context.S timer.S
C_SRC   = main2.c scheduler.c
OBJ     = $(ASM_SRC:.S=.o) $(C_SRC:.c=.o)
ELF     = firmware.elf

# ── Build rules ───────────────────────────────────────────────
all: $(ELF)

$(ELF): $(OBJ)
	$(CC) $(ARCH) -nostdlib -T link.ld -o $@ $^ -lgcc

%.o: %.S
	$(CC) $(AFLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

dump: $(ELF)
	$(OBJDUMP) -d $<

clean:
	rm -f *.o $(ELF)

# ── Run in QEMU ──────────────────────────────────────────────
# Press Ctrl-A then X to exit.
#
# -icount shift=0  : 1 instruction = 1 virtual tick (deterministic)
# sleep=off        : don't throttle to wall-clock time
run: $(ELF)
	qemu-system-riscv32 \
		-machine virt \
		-cpu rv32 \
		-bios none \
		-nographic \
		-icount shift=0,sleep=off \
		-kernel $(ELF)

.PHONY: all dump clean run