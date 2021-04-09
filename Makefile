
#Created by Sambhav Jain
# Email id: sambhav.jv@gmail.com

#   Copyright (C) 2019  IIT Madras. All rights reserved.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.

#	You should have received a copy of the GNU General Public License
#	along with this program.  If not, see <https://www.gnu.org/licenses/>.

SHELL := /bin/bash # Use bash syntax
DC ?=
PROGRAM ?=
#default target board
TARGET ?= artix7_35t
DEBUG ?=
MARCH ?= rv32imac
MABI  ?=
XLEN  ?= 32
FLAGS ?=

all: create_dir
    make serialplotter_bmp280.riscv

serialplotter_bmp280.riscv: create_dir
    @riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) -I$(bspinc) -I$(bspdri) -I$(bspboard) -c ./$(PROGRAM).c -o ./output/$(PROGRAM).o -march=$(MARCH) -mabi=$(MABI) -lm -lgcc
    @echo "Compiled and generated the object file"
    @riscv$(XLEN)-unknown-elf-gcc  -march=$(MARCH) -mabi=$(MABI) -T $(bspboard)/link.ld $(GENLIB)/gen_lib/i2c_driver.o $(GENLIB)/gen_lib/libshakti$(XLEN).a ./output/$(PROGRAM).o -o ./output/$(PROGRAM).shakti -static -nostartfiles -lm -lgcc
    @riscv$(XLEN)-unknown-elf-objdump -D ./output/$(PROGRAM).shakti > ./output/$(PROGRAM).dump

create_dir:
    @mkdir -p ./output




