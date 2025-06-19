const NES_TAG: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
const PRG_ROM_PAGE_SIZE: usize = 16384;
const CHR_ROM_PAGE_SIZE: usize = 8192;

#[derive(Debug, PartialEq)]
pub enum Mirroring {
    Horizontal,
    Vertical,
    FourScreen,
}

pub struct Rom {
    pub prg_rom: Vec<u8>,
    pub chr_rom: Vec<u8>,
    pub mapper: u8,
    pub screen_mirroring: Mirroring,
}

impl Rom {
    pub fn new(raw: &Vec<u8>) -> Result<Rom, String> {
        if &raw[0..4] != NES_TAG {
            return Err("File is not in iNes format".to_string());
        }

        let mapper = (raw[7] & 0b1111_0000) | (raw[6] >> 4);

        let ines_ver = (raw[7] >> 2) & 0b11;
        if ines_ver != 0 {
            return Err("Unsupported iNes version".to_string());
        }

        let four_screen = raw[6] & 0b1000 != 0;
        let vertical_mirroring = raw[6] & 0b1 != 0;

        let screen_mirroring = if four_screen {
            Mirroring::FourScreen
        } else if vertical_mirroring {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };

        let prg_rom_size = raw[4] as usize * PRG_ROM_PAGE_SIZE;
        let chr_rom_size = raw[5] as usize * CHR_ROM_PAGE_SIZE;

        let skip_trainer = raw[6] & 0b100 != 0;

        let prg_room_start = 16 + if skip_trainer { 512 } else { 0 };
        let chr_rom_start = prg_room_start + prg_rom_size;

        Ok(Rom {
            prg_rom: raw[prg_room_start..prg_room_start + prg_rom_size].to_vec(),
            chr_rom: raw[chr_rom_start..chr_rom_start + chr_rom_size].to_vec(),
            mapper: mapper,
            screen_mirroring: screen_mirroring,
        })
    }
}
