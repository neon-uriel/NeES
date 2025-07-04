use crate::cpu::CPU;
use crate::opcodes::Opcode;
pub fn trace(cpu: &CPU) -> String {
    //"0064  A2 01     LDX #$01                        A:01 X:02 Y:03 P:24 SP:FD"
    // Format the CPU state into a string
    let opcode = Opcode::fetch_opcode(cpu);
    let mut output = String::new();
    
    output.push_str(&format!("{:04X} ", cpu.program_counter));
    output.push_str(&format!("{:02X} ", opcode.opcode));
    output.push_str(&format!("A: {:02X} ", cpu.register_a));
    output.push_str(&format!("X: {:02X} ", cpu.register_x));
    output.push_str(&format!("Y: {:02X} ", cpu.register_y));
    output.push_str(&format!("P: {:02X} ", cpu.status));
    output.push_str(&format!("SP: {:02X} ", cpu.stack_pointer));
    //output.push_str(&format!("Cycles: {} ", cpu.cycles));
    
    // Add more details if needed
    // e.g., memory state, registers, etc.
    
    output
}