use crate::opcodes::{self, OpCode};
use std::collections::HashMap;
pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: u8,
    pub program_counter: u16,
    memory: [u8; 0xFFFF],
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

trait Mem {
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
            status: 0,
            program_counter: 0,
            memory: [0; 0xFFFF],
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
        self.status = 0;

        self.program_counter = self.mem_read_u16(0xFFFC);
    }
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    //code writen about opcode
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
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

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    fn iny(&mut self) {
        self.register_y = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }
    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        let data = val.wrapping_add(1);
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.register_a = self.register_a & self.mem_read(addr);
        self.update_zero_and_negative_flags(self.register_a)
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.register_a;
        self.mem_write(addr, data);
    }
    //めっちゃ間違ってるadc
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let val = self.mem_read(addr);
        let carry = self.status & 0b0000_0001;
        let data = self.register_a;
        let (mut res, carrying) = val.overflowing_add(data);
        let mut carrying2;
        (res, carrying2) = val.overflowing_add(carry);
        if carrying || carrying2 {
            self.status |= 0b0000_0001;
        } else {
            self.status = self.status & !(0b0000_0001);
        }
        if val & 0b1000_0000 != 0 && data & 0b1000_0000 != 0 && res & 0b1000_0000 == 0 {
            self.status |= 0b0100_0000;
        } else if val & 0b1000_0000 == 0 && data & 0b1000_0000 == 0 && res & 0b1000_0000 != 0 {
            self.status |= 0b0100_0000;
        }
        self.register_a = res;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.status = self.status | 0b0000_0010;
        } else {
            self.status = self.status & 0b1111_1101;
        }

        if result & 0b1000_0000 != 0 {
            self.status = self.status | 0b1000_0000;
        } else {
            self.status = self.status & 0b0111_1111;
        }
    }

    pub fn run(&mut self) {
        // self.program_counter = 0;
        // I moved init program_counter from here to load function
        let ref opcodes: HashMap<u8, &'static OpCode> = *opcodes::OPCODES_MAP;
        loop {
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;

            let opcode = opcodes
                .get(&code)
                .expect(&format!("OpCode {:x} is not recognized.", code));
            match code {
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(&opcode.mode);
                }
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }
                0xAA => self.tax(),
                0x8A => self.txa(),
                0xA8 => self.tay(),
                0x98 => self.tya(),
                0xE8 => self.inx(),
                0xC8 => self.iny(),
                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(&opcode.mode);
                }
                0x00 => return,
                _ => todo!(),
            }

            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }
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
        assert!(cpu.status & 0b0000_0010 == 0b00);
        assert!(cpu.status & 0b1000_0000 == 0);
    }
    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
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
        assert!(cpu.status & 0b0000_0010 == 0b10);
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
        //cpu.register_x = 0xFF;
        cpu.load_and_run(vec![
            0xA9, 0x01, 0x8d, 0x02, 0x00, 0xa9, 0x0f, 0x6d, 0x02, 0x00, 0x8d, 0x00, 0x00, 0x6d,
            0x00, 0x00, 0x00,
        ]);
        assert_eq!(cpu.register_a, 0x10);
    }
}
