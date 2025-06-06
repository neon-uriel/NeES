use crate::opcodes::{self, OpCode};
use bitflags::{bitflags, Flags};
use std::collections::HashMap;

bitflags! {
    /// # Status Register (P) http://wiki.nesdev.com/w/index.php/Status_flags
    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000; //reserved(予約済)
        const OVERFLOW          = 0b01000000;
        const NEGATIV           = 0b10000000;
    }
}

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: CpuFlags,
    pub program_counter: u16,
    pub stack_pointer: u8,
    memory: [u8; 0x10000], // 64K memory
}

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    // Indirect,
    Indirect_X,
    Indirect_Y,
    Accumulator,
    Relative,
    // Implied,
    NoneAddressing,
}

pub trait Mem {
    fn mem_read(&self, addr: u16) -> u8;

    fn mem_write(&mut self, addr: u16, data: u8);
    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }
}

impl Mem for CPU {
    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: CpuFlags::empty(),
            program_counter: 0,
            stack_pointer: 0,
            memory: [0 as u8; 0x10000],
        }
    }
    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,

            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            // AddressingMode::Indirect => {
            //     let base = self.mem_read(self.program_counter);
            //     let lo = self.mem_read(base as u16);
            //     let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
            //     let deref_base = (hi as u16) << 8 | (lo as u16);
            //     let deref_lo = self.mem_read(deref_base);
            //     let deref_hi = self.mem_read(deref_base.wrapping_add(1));
            //     (deref_hi as u16) << 8 | (deref_lo as u16)
            // }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);
                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
            AddressingMode::Accumulator => self.register_a as u16,
            // AddressingMode::Implied => {
            //     panic!("mode {:?} is not supported.", mode);
            // }
            AddressingMode::Relative => {
                let offset = self.mem_read(self.program_counter);
                let addr = offset.wrapping_add(self.program_counter as u8);
                addr as u16
            }
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported.", mode);
            }
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.status = CpuFlags::empty();

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x0600..(0x0600 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x0600);
    }

    //code writen about opcode
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        let carry_in = if self.status.contains(CpuFlags::CARRY) {
            1
        } else {
            0
        };
        let old_a = self.register_a;
        let sum = old_a as u16 + val as u16 + carry_in as u16;
        let final_res = sum as u8;
        self.register_a = final_res;
        if sum > 0xFF {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        if (old_a ^ final_res) & (val ^ final_res) & 0b1000_0000 != 0 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.register_a = self.register_a & self.mem_read(addr);
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn asl(&mut self, mode: &AddressingMode) {
        let target_addr: Option<u16>;
        let &mut data;
        match mode {
            AddressingMode::Accumulator => {
                data = self.register_a;
                target_addr = None;
            }
            _ => {
                let addr = self.get_operand_address(mode);
                data = self.mem_read(addr);
                target_addr = Some(addr);
            }
        }
        if data & CpuFlags::CARRY.bits() != 0 {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        let res = data << 1;

        match target_addr {
            Some(addr) => {
                self.mem_write(addr, res);
            }
            None => {
                self.register_a = res;
            }
        }
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn bcc(&mut self) {
        self.branch(!self.status.contains(CpuFlags::CARRY));
    }

    fn bcs(&mut self) {
        self.branch(self.status.contains(CpuFlags::CARRY));
    }

    fn beq(&mut self) {
        self.branch(self.status.contains(CpuFlags::ZERO));
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = value & self.register_a;
        if (value & CpuFlags::OVERFLOW.bits()) != 0 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }

        if (value & CpuFlags::NEGATIV.bits()) != 0 {
            self.status.insert(CpuFlags::NEGATIV);
        } else {
            self.status.remove(CpuFlags::NEGATIV);
        }

        if res == 0 {
            self.status.insert(CpuFlags::ZERO);
        } else {
            self.status.remove(CpuFlags::ZERO);
        }
    }

    fn bmi(&mut self) {
        self.branch(self.status.contains(CpuFlags::NEGATIV));
    }
    fn bne(&mut self) {
        self.branch(!self.status.contains(CpuFlags::ZERO));
    }
    fn bpl(&mut self) {
        self.branch(!self.status.contains(CpuFlags::NEGATIV));
    }
    fn bvc(&mut self) {
        self.branch(!self.status.contains(CpuFlags::OVERFLOW));
    }
    fn bvs(&mut self) {
        self.branch(self.status.contains(CpuFlags::OVERFLOW));
    }
    fn clc(&mut self) {
        self.status.remove(CpuFlags::CARRY);
    }
    fn cld(&mut self) {
        self.status.remove(CpuFlags::DECIMAL_MODE);
    }
    fn cli(&mut self) {
        self.status.remove(CpuFlags::INTERRUPT_DISABLE);
    }
    fn clv(&mut self) {
        self.status.remove(CpuFlags::OVERFLOW);
    }
    fn cmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mem_data = self.mem_read(addr);
        self.compare(self.register_a, mem_data);
    }
    fn cpx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mem_data = self.mem_read(addr);
        self.compare(self.register_x, mem_data);
    }
    fn cpy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mem_data = self.mem_read(addr);
        self.compare(self.register_y, mem_data);
    }
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        let res = data.wrapping_sub(1);
        self.update_zero_and_negative_flags(res);
        self.mem_write(addr, res);
    }
    fn dex(&mut self) {
        let data = self.register_x;
        let res = data.wrapping_sub(1);
        self.update_zero_and_negative_flags(res);
        self.register_x = res;
    }
    fn dey(&mut self) {
        let data = self.register_y;
        let res = data.wrapping_sub(1);
        self.update_zero_and_negative_flags(res);
        self.register_y = res;
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = self.register_a ^ value;
        self.update_zero_and_negative_flags(res);
        self.register_a = res;
    }

    fn pha(&mut self) {
        //あやしい
        self.mem_write(0x0100 as u16 + self.stack_pointer as u16, self.register_a);
        self.stack_pointer -= 1;
    }

    fn php(&mut self) {
        //あやしい
        self.mem_write(
            0x0100 as u16 + self.stack_pointer as u16,
            self.status.bits(),
        );
        self.stack_pointer -= 1;
    }

    fn pla(&mut self) {
        //あやしい
        self.stack_pointer += 1;
        self.register_a = self.mem_read(0x0100 as u16 + self.stack_pointer as u16);
    }

    fn plp(&mut self) {
        self.stack_pointer += 1;
        self.status =
            CpuFlags::from_bits_truncate(self.mem_read(0x0100 as u16 + self.stack_pointer as u16));
    }

    fn rol(&mut self, mode: &AddressingMode) {
        let &mut data;
        let &mut target_addr;
        match mode {
            AddressingMode::Accumulator => {
                data = self.register_a;
                target_addr = None;
            }
            _ => {
                let addr = self.get_operand_address(mode);
                data = self.mem_read(addr);
                target_addr = Some(addr);
            }
        }
        if (data & 0b1000_0000 > 1) {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        let res = data << 1;
        self.update_zero_and_negative_flags(res);
        match target_addr {
            Some(addr) => {
                self.mem_write(addr, res);
            }
            None => {
                self.register_a = res;
            }
        }
    }

    fn ror(&mut self, mode: &AddressingMode) {
        let &mut data;
        let &mut target_addr;
        match mode {
            AddressingMode::Accumulator => {
                data = self.register_a;
                target_addr = None;
            }
            _ => {
                let addr = self.get_operand_address(mode);
                data = self.mem_read(addr);
                target_addr = Some(addr);
            }
        }
        if (data & 0b0000_0001) > 0 {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        let res = data >> 1;
        self.update_zero_and_negative_flags(res);
        match target_addr {
            Some(addr) => {
                self.mem_write(addr, res);
            }
            None => {
                self.register_a = res;
            }
        }
    }

    fn rti(&mut self) {
        self.status = CpuFlags::from_bits_truncate(self.stack_pointer);
        self.program_counter = self.stack_pointer as u16;
    }

    fn rts(&mut self) {
        self.program_counter = self.stack_pointer as u16;
        self.program_counter += 1;
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn lsr(&mut self, mode: &AddressingMode) {
        let target_addr: Option<u16>;
        let mut data;
        match mode {
            AddressingMode::Accumulator => {
                data = self.register_a;
                target_addr = None;
            }
            _ => {
                let addr = self.get_operand_address(mode);
                data = self.mem_read(addr);
                target_addr = Some(addr);
            }
        }
        if data & CpuFlags::CARRY.bits() != 0 {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        let res = data >> 1;
        match target_addr {
            Some(addr) => {
                self.mem_write(addr, res);
                self.mem_write(addr, res);
            }
            None => {
                self.register_a = res;
            }
        }
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.register_a = self.register_a | self.mem_read(addr);
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn sec(&mut self) {
        self.status.insert(CpuFlags::CARRY);
    }

    fn sed(&mut self) {
        self.status.insert(CpuFlags::DECIMAL_MODE);
    }

    fn sei(&mut self) {
        self.status.insert(CpuFlags::INTERRUPT_DISABLE);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let C = self.status.contains(CpuFlags::CARRY) as u8;
        let res = self.register_a - value - (1 - C);
        if !(res < 0) {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        if ((res ^ self.register_a) & (res ^ !value) & 0x80) > 1 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }
        self.update_zero_and_negative_flags(res);
        self.register_a = res;
    }

    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }
    fn tay(&mut self) {
        self.register_y = self.register_a;
        self.update_zero_and_negative_flags(self.register_y);
    }
    fn tya(&mut self) {
        self.register_a = self.register_y;
        self.update_zero_and_negative_flags(self.register_a);
    }
    fn txa(&mut self) {
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn tsx(&mut self) {
        self.register_x = self.stack_pointer;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn txs(&mut self) {
        self.stack_pointer = self.register_x;
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }
    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        let data = val.wrapping_add(1);
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }

    fn nop(&mut self) {
        return;
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        //program_counterの下位8bitを渡すのでいいのか？？
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        self.program_counter = val as u16;
    }

    fn jsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        self.stack_pointer = self.program_counter as u8 + 2;
        self.program_counter = val as u16;
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.status |= CpuFlags::ZERO;
        } else {
            self.status &= !CpuFlags::ZERO;
        }

        if result & 0b1000_0000 != 0 {
            self.status |= CpuFlags::NEGATIV;
        } else {
            self.status &= !CpuFlags::NEGATIV;
        }
    }

    fn branch(&mut self, condition: bool) {
        if (condition) {
            let jump = self.mem_read(self.program_counter) as i8;
            let jump_addr = self
                .program_counter
                .wrapping_add(1)
                .wrapping_add(jump as u16);
            self.program_counter = jump_addr;
        }
    }

    fn compare(&mut self, reg_data: u8, mem_data: u8) {
        let res = reg_data - mem_data;
        if (res >= 0) {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        self.update_zero_and_negative_flags(res);
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
        // self.program_counter = 0;
        // I moved init program_counter from here to load function
        // let ref opcodes: HashMap<u8, &'static OpCode> = *opcodes::OPCODES_MAP;

        // loop {
        // let code = self.mem_read(self.program_counter);
        // self.program_counter += 1;
        // let program_counter_state = self.program_counter;

        // let opcode = opcodes
        //     .get(&code)
        //     .expect(&format!("OpCode {:x} is not recognized.", code));
        //     match code {
        //         0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
        //             self.lda(&opcode.mode);
        //         }
        //         0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
        //             self.ldx(&opcode.mode);
        //         }
        //         0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
        //             self.ldy(&opcode.mode);
        //         }
        //         0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
        //             self.lsr(&opcode.mode);
        //         }
        //         0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
        //             self.sta(&opcode.mode);
        //         }
        //         0x86 | 0x96 | 0x8E => {
        //             self.stx(&opcode.mode);
        //         }
        //         0x84 | 0x94 | 0x8C => {
        //             self.sty(&opcode.mode);
        //         }
        //         0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
        //             self.and(&opcode.mode);
        //         }
        //         0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
        //             self.ora(&opcode.mode);
        //         }
        //         0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
        //             self.adc(&opcode.mode);
        //         }
        //         0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
        //             self.asl(&opcode.mode);
        //         }
        //         0x24 | 0x2C => {
        //             self.bit(&opcode.mode);
        //         }
        //         0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
        //             self.cmp(&opcode.mode);
        //         }
        //         0xE0 | 0xE4 | 0xEC => {
        //             self.cpx(&opcode.mode);
        //         }
        //         0xC0 | 0xC4 | 0xCC => {
        //             self.cpy(&opcode.mode);
        //         }
        //         0xC6 | 0xD6 | 0xCE | 0xDE => {
        //             self.dec(&opcode.mode);
        //         }
        //         0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
        //             self.eor(&opcode.mode);
        //         }
        //         0x4C | 0x6C => {
        //             self.jmp(&opcode.mode);
        //         }
        //         0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
        //             self.sbc(&opcode.mode);
        //         }
        //         0x38 => self.sec(),
        //         0xF8 => self.sed(),
        //         0x78 => self.sei(),
        //         0x40 => self.rti(),
        //         0x60 => self.rts(),
        //         0x48 => self.pha(),
        //         0x08 => self.php(),
        //         0x68 => self.pla(),
        //         0x28 => self.plp(),
        //         0x20 => self.jsr(&opcode.mode),
        //         0xCA => self.dex(),
        //         0x88 => self.dey(),
        //         0x90 => self.bcc(),
        //         0xB0 => self.bcs(),
        //         0xF0 => self.beq(),
        //         0x20 => self.bmi(),
        //         0xD0 => self.bne(),
        //         0x10 => self.bpl(),
        //         0x50 => self.bvc(),
        //         0x70 => self.bvs(),
        //         0x18 => self.clc(),
        //         0x58 => self.cli(),
        //         0xD8 => self.cld(),
        //         0x58 => self.cli(),
        //         0xB8 => self.clv(),
        //         0xAA => self.tax(),
        //         0x8A => self.txa(),
        //         0xA8 => self.tay(),
        //         0x98 => self.tya(),
        //         0xBA => self.tsx(),
        //         0x9A => self.txs(),
        //         0xE8 => self.inx(),
        //         0xC8 => self.iny(),
        //         0xEA => self.nop(),
        //         0xE6 | 0xF6 | 0xEE | 0xFE => {
        //             self.inc(&opcode.mode);
        //         }
        //         0x00 => return,
        //         _ => todo!(),
        //     }

        // if program_counter_state == self.program_counter {
        //     self.program_counter += (opcode.len - 1) as u16;
        // }
        // }
    }
    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;
        loop {
            println!("デンシャルル");
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;
            let opcode = opcodes.get(&code).unwrap();
            // let opcode = opcodes
            //     .get(&code)
            //     .expect(&format!("OpCode {:x} is not recognized.", code));
            match code {
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(&opcode.mode);
                }
                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(&opcode.mode);
                }
                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(&opcode.mode);
                }
                0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(&opcode.mode);
                }
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }
                0x86 | 0x96 | 0x8E => {
                    self.stx(&opcode.mode);
                }
                0x84 | 0x94 | 0x8C => {
                    self.sty(&opcode.mode);
                }
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }
                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }
                0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(&opcode.mode);
                }
                0x24 | 0x2C => {
                    self.bit(&opcode.mode);
                }
                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.cmp(&opcode.mode);
                }
                0xE0 | 0xE4 | 0xEC => {
                    self.cpx(&opcode.mode);
                }
                0xC0 | 0xC4 | 0xCC => {
                    self.cpy(&opcode.mode);
                }
                0xC6 | 0xD6 | 0xCE | 0xDE => {
                    self.dec(&opcode.mode);
                }
                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }
                0x4C | 0x6C => {
                    self.jmp(&opcode.mode);
                }
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }
                0x38 => self.sec(),
                0xF8 => self.sed(),
                0x78 => self.sei(),
                0x40 => self.rti(),
                0x60 => self.rts(),
                0x48 => self.pha(),
                0x08 => self.php(),
                0x68 => self.pla(),
                0x28 => self.plp(),
                0x20 => self.jsr(&opcode.mode),
                0xCA => self.dex(),
                0x88 => self.dey(),
                0x90 => self.bcc(),
                0xB0 => self.bcs(),
                0xF0 => self.beq(),
                0x20 => self.bmi(),
                0xD0 => self.bne(),
                0x10 => self.bpl(),
                0x50 => self.bvc(),
                0x70 => self.bvs(),
                0x18 => self.clc(),
                0x58 => self.cli(),
                0xD8 => self.cld(),
                0x58 => self.cli(),
                0xB8 => self.clv(),
                0xAA => self.tax(),
                0x8A => self.txa(),
                0xA8 => self.tay(),
                0x98 => self.tya(),
                0xBA => self.tsx(),
                0x9A => self.txs(),
                0xE8 => self.inx(),
                0xC8 => self.iny(),
                0xEA => self.nop(),
                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(&opcode.mode);
                }
                0x00 => return,
                _ => todo!(),
            }
            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }
            callback(self);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 0x05);
        assert!(!cpu.status.contains(CpuFlags::ZERO));
        assert!(!cpu.status.contains(CpuFlags::NEGATIV));
    }
    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x00, 0x00]);
        assert!(cpu.status.contains(CpuFlags::ZERO));
    }
    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x0A, 0xAA, 0x00]);
        assert_eq!(cpu.register_x, 10);
    }
    #[test]
    fn test_0xe8_inx_increment_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x0A, 0xAA, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 11);
    }
    #[test]
    fn test_5_ops_working_togather() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0xC0, 0xAA, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 0xC1);
    }
    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        //cpu.register_x = 0xFF;
        cpu.load_and_run(vec![0xA9, 0xFF, 0xAA, 0xE8, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1);
    }
    #[test]
    fn test_and_zeroflag() {
        let mut cpu = CPU::new();
        //cpu.register_x = 0xFF;
        cpu.load_and_run(vec![0xA9, 0x00, 0x29, 0x01]);
        assert!(cpu.status.contains(CpuFlags::ZERO));
    }
    #[test]
    fn test_0xa9_and() {
        let mut cpu = CPU::new();
        //cpu.register_x = 0xFF;
        cpu.load_and_run(vec![0xA9, 0xFF, 0x29, 0x01]);
        assert_eq!(cpu.register_a, 0x01);
    }
    #[test]
    fn test_0x6d_adc() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![
            // 0x8000: 0xA9, 0x01   ; LDA #$01   -> A = 0x01
            // 0x8002: 0x8D, 0x02, 0x00 ; STA $0002  -> メモリ[0x0002] = A (0x01)
            // 0x8005: 0xA9, 0x0F   ; LDA #$0F   -> A = 0x0F
            // 0x8007: 0x6D, 0x02, 0x00 ; ADC $0002  -> A = 0x0F + 0x01 + 0 = 0x10
            // 0x800A: 0x00         ; BRK (ここでプログラム終了)
            0xA9, 0x01, 0x8d, 0x02, 0x00, 0xa9, 0x0f, 0x6d, 0x02, 0x00, 0x00, // ここが修正
        ]);
        assert_eq!(cpu.register_a, 0x10); // Aレジスタが0x10になっていることを確認
    }
}
