use display_interface::{DataFormat::U8, DisplayError, WriteOnlyDataCommand};

use super::brightness::Brightness;
use super::command::{AddrMode, Command, Page, VcomhLevel};
use super::{rotation::DisplayRotation, size::DisplaySize};

/// Errors which can occur when interacting with the terminal mode
#[derive(Clone)]
pub enum TerminalError {
    /// An error occurred in the underlying interface layer
    InterfaceError(DisplayError),
    /// The mode was used before it was initialized
    Uninitialized,
    /// A location was specified outside the bounds of the screen
    OutOfBounds,
}

impl core::fmt::Debug for TerminalError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        match self {
            Self::InterfaceError(_) => "InterfaceError".fmt(f),
            Self::Uninitialized => "Uninitialized".fmt(f),
            Self::OutOfBounds => "OutOfBound".fmt(f),
        }
    }
}

// Cannot use From<_> due to coherence
trait IntoTerminalModeResult<T> {
    fn terminal_err(self) -> Result<T, TerminalError>;
}

impl<T> IntoTerminalModeResult<T> for Result<T, DisplayError> {
    fn terminal_err(self) -> Result<T, TerminalError> {
        self.map_err(TerminalError::InterfaceError)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Ssd1306<DI, SIZE> {
    pub interface: DI,
    size: SIZE,
    addr_mode: AddrMode,
    rotation: DisplayRotation,
    current_x: u8,
    current_y: u8,
}

impl<DI, SIZE> Ssd1306<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: DisplaySize,
{
    pub fn new(interface: DI, size: SIZE, rotation: DisplayRotation) -> Self {
        Self {
            interface,
            size,
            addr_mode: AddrMode::Page,
            rotation,
            current_x: 0,
            current_y: 0,
        }
    }

    /// Clear the display.
    pub fn clear(&mut self) -> Result<(), DisplayError> {
        const DATA: [u8; 1] = [0x00];

        for _i in 0..(SIZE::HEIGHT / 8) {
            Command::PageStart(Page::from(_i * 8)).send(&mut self.interface)?;
            Command::LowerColStart(0x00).send(&mut self.interface)?;
            Command::UpperColStart(0x10).send(&mut self.interface)?;

            for _k in 0..SIZE::WIDTH {
                self.interface.send_data(U8(&DATA))?;
            }
        }
        Ok(())
    }

    /// Set the display rotation.
    fn set_rotation(&mut self, rotation: DisplayRotation) -> Result<(), DisplayError> {
        self.rotation = rotation;

        match rotation {
            DisplayRotation::Rotate0 => {
                Command::SegmentRemap(true).send(&mut self.interface)?;
                Command::ReverseComDir(true).send(&mut self.interface)?;
            }
            DisplayRotation::Rotate90 => {
                Command::SegmentRemap(false).send(&mut self.interface)?;
                Command::ReverseComDir(true).send(&mut self.interface)?;
            }
            DisplayRotation::Rotate180 => {
                Command::SegmentRemap(false).send(&mut self.interface)?;
                Command::ReverseComDir(false).send(&mut self.interface)?;
            }
            DisplayRotation::Rotate270 => {
                Command::SegmentRemap(true).send(&mut self.interface)?;
                Command::ReverseComDir(false).send(&mut self.interface)?;
            }
        };

        Ok(())
    }
    /// Initialise in page addressing mode.
    pub fn init(&mut self) -> Result<(), DisplayError> {
        self.init_with_addr_mode(AddrMode::Page)?;
        Ok(())
    }

    /// Initialise the display in one of the available addressing modes.
    pub fn init_with_addr_mode(&mut self, mode: AddrMode) -> Result<(), DisplayError> {
        let rotation = self.rotation;

        Command::DisplayOn(false).send(&mut self.interface)?;
        Command::DisplayClockDiv(0x8, 0x0).send(&mut self.interface)?;
        Command::Multiplex(SIZE::HEIGHT - 1).send(&mut self.interface)?;
        Command::DisplayOffset(0).send(&mut self.interface)?;
        Command::StartLine(0).send(&mut self.interface)?;
        // TODO: Ability to turn charge pump on/off
        Command::ChargePump(true).send(&mut self.interface)?;
        Command::AddressMode(mode).send(&mut self.interface)?;

        self.size.configure(&mut self.interface)?;
        self.set_rotation(rotation)?;

        self.set_brightness(Brightness::default())?;
        Command::VcomhDeselect(VcomhLevel::Auto).send(&mut self.interface)?;
        Command::AllOn(false).send(&mut self.interface)?;
        Command::Invert(false).send(&mut self.interface)?;
        Command::EnableScroll(false).send(&mut self.interface)?;
        //Command::DisplayOn(true).send(&mut self.interface)?;

        self.addr_mode = mode;

        Ok(())
    }

    pub fn display_on(&mut self, on: bool) -> Result<(), DisplayError> {
        Command::DisplayOn(on).send(&mut self.interface)
    }

    /// Change the display brightness.
    pub fn set_brightness(&mut self, brightness: Brightness) -> Result<(), DisplayError> {
        // Should be moved to Brightness::new once conditions can be used in const functions
        debug_assert!(
            0 < brightness.precharge && brightness.precharge <= 15,
            "Precharge value must be between 1 and 15"
        );

        Command::PreChargePeriod(1, brightness.precharge).send(&mut self.interface)?;
        Command::Contrast(brightness.contrast).send(&mut self.interface)
    }

    pub fn write_char(&mut self, c: char) -> Result<(), DisplayError> {
        match c {
            '\n' => {}
            '\r' => {}
            _ => {
                let mut data: [u8; 1];
                let num_rows = 4; //(26 / 8).ceil();
                let bitmap = self.char_to_bitmap(c);

                for i in 0..num_rows {
                    Command::PageStart(Page::from(i * 8 + self.current_y))
                        .send(&mut self.interface)?;
                    Command::ColumnAddress(self.current_x, self.current_x + 16)
                        .send(&mut self.interface)?;

                    for j in 0..16 {
                        let mut b = 0;
                        let mut idx = i * 8;

                        for k in 0..8 {
                            if idx >= 26 {
                                break;
                            }

                            let mut mask = bitmap[idx as usize];
                            mask &= 0x01 << (15 - j);
                            mask = mask >> (15 - j);
                            mask = mask << k;
                            b |= mask as u8;
                            idx += 1;
                        }

                        data = [b];
                        self.interface.send_data(U8(&data))?;
                    }
                }
            }
        }
        Ok(())
    }

    pub fn set_position(&mut self, col: u8, row: u8) -> &mut Self {
        self.current_x = col;
        self.current_y = row;
        self
    }

    fn char_to_bitmap(&mut self, input: char) -> [u16; 26] {
        // Populate the array with the data from the character array at the right index
        match input {
            '!' => [
                0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03C0, 0x03C0,
                0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '"' => [
                0x1E3C, 0x1E3C, 0x1E3C, 0x1E3C, 0x1E3C, 0x1E3C, 0x1E3C, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '#' => [
                0x01CE, 0x03CE, 0x03DE, 0x039E, 0x039C, 0x079C, 0x3FFF, 0x7FFF, 0x0738, 0x0F38,
                0x0F78, 0x0F78, 0x0E78, 0xFFFF, 0xFFFF, 0x1EF0, 0x1CF0, 0x1CE0, 0x3CE0, 0x3DE0,
                0x39E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '$' => [
                0x03FC, 0x0FFE, 0x1FEE, 0x1EE0, 0x1EE0, 0x1EE0, 0x1EE0, 0x1FE0, 0x0FE0, 0x07E0,
                0x03F0, 0x01FC, 0x01FE, 0x01FE, 0x01FE, 0x01FE, 0x01FE, 0x01FE, 0x3DFE, 0x3FFC,
                0x0FF0, 0x01E0, 0x01E0, 0x0000, 0x0000, 0x0000,
            ],
            '%' => [
                0x3E03, 0xF707, 0xE78F, 0xE78E, 0xE39E, 0xE3BC, 0xE7B8, 0xE7F8, 0xF7F0, 0x3FE0,
                0x01C0, 0x03FF, 0x07FF, 0x07F3, 0x0FF3, 0x1EF3, 0x3CF3, 0x38F3, 0x78F3, 0xF07F,
                0xE03F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '&' => [
                0x07E0, 0x0FF8, 0x0F78, 0x1F78, 0x1F78, 0x1F78, 0x0F78, 0x0FF0, 0x0FE0, 0x1F80,
                0x7FC3, 0xFBC3, 0xF3E7, 0xF1F7, 0xF0F7, 0xF0FF, 0xF07F, 0xF83E, 0x7C7F, 0x3FFF,
                0x1FEF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '\'' => [
                0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03C0, 0x01C0, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '(' => [
                0x003F, 0x007C, 0x01F0, 0x01E0, 0x03C0, 0x07C0, 0x0780, 0x0780, 0x0F80, 0x0F00,
                0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F80, 0x0780, 0x0780, 0x07C0, 0x03C0,
                0x01E0, 0x01F0, 0x007C, 0x003F, 0x000F, 0x0000,
            ],
            ')' => [
                0x7E00, 0x1F00, 0x07C0, 0x03C0, 0x01E0, 0x01F0, 0x00F0, 0x00F0, 0x00F8, 0x0078,
                0x0078, 0x0078, 0x0078, 0x0078, 0x0078, 0x00F8, 0x00F0, 0x00F0, 0x01F0, 0x01E0,
                0x03C0, 0x07C0, 0x1F00, 0x7E00, 0x7800, 0x0000,
            ],
            '*' => [
                0x03E0, 0x03C0, 0x01C0, 0x39CE, 0x3FFF, 0x3F7F, 0x0320, 0x0370, 0x07F8, 0x0F78,
                0x1F3C, 0x0638, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '+' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x01C0, 0x01C0, 0x01C0, 0x01C0,
                0x01C0, 0x01C0, 0x01C0, 0xFFFF, 0xFFFF, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0,
                0x01C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            ',' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x01E0, 0x01E0, 0x01E0, 0x01C0, 0x0380,
            ],
            '-' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x3FFE, 0x3FFE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '.' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '/' => [
                0x000F, 0x000F, 0x001E, 0x001E, 0x003C, 0x003C, 0x0078, 0x0078, 0x00F0, 0x00F0,
                0x01E0, 0x01E0, 0x03C0, 0x03C0, 0x0780, 0x0780, 0x0F00, 0x0F00, 0x1E00, 0x1E00,
                0x3C00, 0x3C00, 0x7800, 0x7800, 0xF000, 0x0000,
            ],
            '0' => [
                0x07F0, 0x0FF8, 0x1F7C, 0x3E3E, 0x3C1E, 0x7C1F, 0x7C1F, 0x780F, 0x780F, 0x780F,
                0x780F, 0x780F, 0x780F, 0x780F, 0x7C1F, 0x7C1F, 0x3C1E, 0x3E3E, 0x1F7C, 0x0FF8,
                0x07F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '1' => [
                0x00F0, 0x07F0, 0x3FF0, 0x3FF0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0,
                0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x3FFF,
                0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '2' => [
                0x0FE0, 0x3FF8, 0x3C7C, 0x003C, 0x003E, 0x003E, 0x003E, 0x003C, 0x003C, 0x007C,
                0x00F8, 0x01F0, 0x03E0, 0x07C0, 0x0780, 0x0F00, 0x1E00, 0x3E00, 0x3C00, 0x3FFE,
                0x3FFE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '3' => [
                0x0FF0, 0x1FF8, 0x1C7C, 0x003E, 0x003E, 0x003E, 0x003C, 0x003C, 0x00F8, 0x0FF0,
                0x0FF8, 0x007C, 0x003E, 0x001E, 0x001E, 0x001E, 0x001E, 0x003E, 0x1C7C, 0x1FF8,
                0x1FE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '4' => [
                0x0078, 0x00F8, 0x00F8, 0x01F8, 0x03F8, 0x07F8, 0x07F8, 0x0F78, 0x1E78, 0x1E78,
                0x3C78, 0x7878, 0x7878, 0xFFFF, 0xFFFF, 0x0078, 0x0078, 0x0078, 0x0078, 0x0078,
                0x0078, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '5' => [
                0x1FFC, 0x1FFC, 0x1FFC, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1FE0, 0x1FF8,
                0x00FC, 0x007C, 0x003E, 0x003E, 0x001E, 0x003E, 0x003E, 0x003C, 0x1C7C, 0x1FF8,
                0x1FE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '6' => [
                0x01FC, 0x07FE, 0x0F8E, 0x1F00, 0x1E00, 0x3E00, 0x3C00, 0x3C00, 0x3DF8, 0x3FFC,
                0x7F3E, 0x7E1F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3E0F, 0x1E1F, 0x1F3E, 0x0FFC,
                0x03F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '7' => [
                0x3FFF, 0x3FFF, 0x3FFF, 0x000F, 0x001E, 0x001E, 0x003C, 0x0038, 0x0078, 0x00F0,
                0x00F0, 0x01E0, 0x01E0, 0x03C0, 0x03C0, 0x0780, 0x0F80, 0x0F80, 0x0F00, 0x1F00,
                0x1F00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '8' => [
                0x07F8, 0x0FFC, 0x1F3E, 0x1E1E, 0x3E1E, 0x3E1E, 0x1E1E, 0x1F3C, 0x0FF8, 0x07F0,
                0x0FF8, 0x1EFC, 0x3E3E, 0x3C1F, 0x7C1F, 0x7C0F, 0x7C0F, 0x3C1F, 0x3F3E, 0x1FFC,
                0x07F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '9' => [
                0x07F0, 0x0FF8, 0x1E7C, 0x3C3E, 0x3C1E, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x3C1F,
                0x3E3F, 0x1FFF, 0x07EF, 0x001F, 0x001E, 0x001E, 0x003E, 0x003C, 0x38F8, 0x3FF0,
                0x1FE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            ':' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            ';' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x01E0, 0x01E0, 0x01E0, 0x03C0, 0x0380,
            ],
            '<' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x000F, 0x003F, 0x00FC,
                0x03F0, 0x0FC0, 0x3F00, 0xFE00, 0x3F00, 0x0FC0, 0x03F0, 0x00FC, 0x003F, 0x000F,
                0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '=' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '>' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xE000, 0xF800, 0x7E00, 0x1F80,
                0x07E0, 0x01F8, 0x007E, 0x001F, 0x007E, 0x01F8, 0x07E0, 0x1F80, 0x7E00, 0xF800,
                0xE000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '?' => [
                0x1FF0, 0x3FFC, 0x383E, 0x381F, 0x381F, 0x001E, 0x001E, 0x003C, 0x0078, 0x00F0,
                0x01E0, 0x03C0, 0x03C0, 0x07C0, 0x07C0, 0x0000, 0x0000, 0x0000, 0x07C0, 0x07C0,
                0x07C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '@' => [
                0x03F8, 0x0FFE, 0x1F1E, 0x3E0F, 0x3C7F, 0x78FF, 0x79EF, 0x73C7, 0xF3C7, 0xF38F,
                0xF38F, 0xF38F, 0xF39F, 0xF39F, 0x73FF, 0x7BFF, 0x79F7, 0x3C00, 0x1F1C, 0x0FFC,
                0x03F8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'A' => [
                0x0000, 0x0000, 0x0000, 0x03E0, 0x03E0, 0x07F0, 0x07F0, 0x07F0, 0x0F78, 0x0F78,
                0x0E7C, 0x1E3C, 0x1E3C, 0x3C3E, 0x3FFE, 0x3FFF, 0x781F, 0x780F, 0xF00F, 0xF007,
                0xF007, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'B' => [
                0x0000, 0x0000, 0x0000, 0x3FF8, 0x3FFC, 0x3C3E, 0x3C1E, 0x3C1E, 0x3C1E, 0x3C3E,
                0x3C7C, 0x3FF0, 0x3FF8, 0x3C7E, 0x3C1F, 0x3C1F, 0x3C0F, 0x3C0F, 0x3C1F, 0x3FFE,
                0x3FF8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'C' => [
                0x0000, 0x0000, 0x0000, 0x01FF, 0x07FF, 0x1F87, 0x3E00, 0x3C00, 0x7C00, 0x7800,
                0x7800, 0x7800, 0x7800, 0x7800, 0x7C00, 0x7C00, 0x3E00, 0x3F00, 0x1F83, 0x07FF,
                0x01FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'D' => [
                0x0000, 0x0000, 0x0000, 0x7FF0, 0x7FFC, 0x787E, 0x781F, 0x781F, 0x780F, 0x780F,
                0x780F, 0x780F, 0x780F, 0x780F, 0x780F, 0x780F, 0x781F, 0x781E, 0x787E, 0x7FF8,
                0x7FE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'E' => [
                0x0000, 0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00,
                0x3E00, 0x3FFE, 0x3FFE, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3FFF,
                0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'F' => [
                0x0000, 0x0000, 0x0000, 0x1FFF, 0x1FFF, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1E00,
                0x1E00, 0x1FFF, 0x1FFF, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1E00, 0x1E00,
                0x1E00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'G' => [
                0x0000, 0x0000, 0x0000, 0x03FE, 0x0FFF, 0x1F87, 0x3E00, 0x7C00, 0x7C00, 0x7800,
                0xF800, 0xF800, 0xF87F, 0xF87F, 0x780F, 0x7C0F, 0x7C0F, 0x3E0F, 0x1F8F, 0x0FFF,
                0x03FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'H' => [
                0x0000, 0x0000, 0x0000, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F,
                0x7C1F, 0x7FFF, 0x7FFF, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F, 0x7C1F,
                0x7C1F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'I' => [
                0x0000, 0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x3FFF,
                0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'J' => [
                0x0000, 0x0000, 0x0000, 0x1FFC, 0x1FFC, 0x007C, 0x007C, 0x007C, 0x007C, 0x007C,
                0x007C, 0x007C, 0x007C, 0x007C, 0x007C, 0x007C, 0x0078, 0x0078, 0x38F8, 0x3FF0,
                0x3FC0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'K' => [
                0x0000, 0x0000, 0x0000, 0x3C1F, 0x3C1E, 0x3C3C, 0x3C78, 0x3CF0, 0x3DE0, 0x3FE0,
                0x3FC0, 0x3F80, 0x3FC0, 0x3FE0, 0x3DF0, 0x3CF0, 0x3C78, 0x3C7C, 0x3C3E, 0x3C1F,
                0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'L' => [
                0x0000, 0x0000, 0x0000, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00,
                0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3FFF,
                0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'M' => [
                0x0000, 0x0000, 0x0000, 0xF81F, 0xFC1F, 0xFC1F, 0xFE3F, 0xFE3F, 0xFE3F, 0xFF7F,
                0xFF77, 0xFF77, 0xF7F7, 0xF7E7, 0xF3E7, 0xF3E7, 0xF3C7, 0xF007, 0xF007, 0xF007,
                0xF007, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'N' => [
                0x0000, 0x0000, 0x0000, 0x7C0F, 0x7C0F, 0x7E0F, 0x7F0F, 0x7F0F, 0x7F8F, 0x7F8F,
                0x7FCF, 0x7BEF, 0x79EF, 0x79FF, 0x78FF, 0x78FF, 0x787F, 0x783F, 0x783F, 0x781F,
                0x781F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'O' => [
                0x0000, 0x0000, 0x0000, 0x07F0, 0x1FFC, 0x3E3E, 0x7C1F, 0x780F, 0x780F, 0xF80F,
                0xF80F, 0xF80F, 0xF80F, 0xF80F, 0xF80F, 0x780F, 0x780F, 0x7C1F, 0x3E3E, 0x1FFC,
                0x07F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'P' => [
                0x0000, 0x0000, 0x0000, 0x3FFC, 0x3FFF, 0x3E1F, 0x3E0F, 0x3E0F, 0x3E0F, 0x3E0F,
                0x3E1F, 0x3E3F, 0x3FFC, 0x3FF0, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00, 0x3E00,
                0x3E00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'Q' => [
                0x0000, 0x0000, 0x0000, 0x07F0, 0x1FFC, 0x3E3E, 0x7C1F, 0x780F, 0x780F, 0xF80F,
                0xF80F, 0xF80F, 0xF80F, 0xF80F, 0xF80F, 0x780F, 0x780F, 0x7C1F, 0x3E3E, 0x1FFC,
                0x07F8, 0x007C, 0x003F, 0x000F, 0x0003, 0x0000,
            ],
            'R' => [
                0x0000, 0x0000, 0x0000, 0x3FF0, 0x3FFC, 0x3C7E, 0x3C3E, 0x3C1E, 0x3C1E, 0x3C3E,
                0x3C3C, 0x3CFC, 0x3FF0, 0x3FE0, 0x3DF0, 0x3CF8, 0x3C7C, 0x3C3E, 0x3C1E, 0x3C1F,
                0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'S' => [
                0x0000, 0x0000, 0x0000, 0x07FC, 0x1FFE, 0x3E0E, 0x3C00, 0x3C00, 0x3C00, 0x3E00,
                0x1FC0, 0x0FF8, 0x03FE, 0x007F, 0x001F, 0x000F, 0x000F, 0x201F, 0x3C3E, 0x3FFC,
                0x1FF0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'T' => [
                0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'U' => [
                0x0000, 0x0000, 0x0000, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F,
                0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x7C0F, 0x3C1E, 0x3C1E, 0x3E3E, 0x1FFC,
                0x07F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'V' => [
                0x0000, 0x0000, 0x0000, 0xF007, 0xF007, 0xF807, 0x780F, 0x7C0F, 0x3C1E, 0x3C1E,
                0x3E1E, 0x1E3C, 0x1F3C, 0x1F78, 0x0F78, 0x0FF8, 0x07F0, 0x07F0, 0x07F0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'W' => [
                0x0000, 0x0000, 0x0000, 0xE003, 0xF003, 0xF003, 0xF007, 0xF3E7, 0xF3E7, 0xF3E7,
                0x73E7, 0x7BF7, 0x7FF7, 0x7FFF, 0x7F7F, 0x7F7F, 0x7F7E, 0x3F7E, 0x3E3E, 0x3E3E,
                0x3E3E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'X' => [
                0x0000, 0x0000, 0x0000, 0xF807, 0x7C0F, 0x3E1E, 0x3E3E, 0x1F3C, 0x0FF8, 0x07F0,
                0x07E0, 0x03E0, 0x03E0, 0x07F0, 0x0FF8, 0x0F7C, 0x1E7C, 0x3C3E, 0x781F, 0x780F,
                0xF00F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'Y' => [
                0x0000, 0x0000, 0x0000, 0xF807, 0x7807, 0x7C0F, 0x3C1E, 0x3E1E, 0x1F3C, 0x0F78,
                0x0FF8, 0x07F0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'Z' => [
                0x0000, 0x0000, 0x0000, 0x7FFF, 0x7FFF, 0x000F, 0x001F, 0x003E, 0x007C, 0x00F8,
                0x00F0, 0x01E0, 0x03E0, 0x07C0, 0x0F80, 0x0F00, 0x1E00, 0x3E00, 0x7C00, 0x7FFF,
                0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '[' => [
                0x07FF, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780,
                0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780,
                0x0780, 0x0780, 0x0780, 0x07FF, 0x07FF, 0x0000,
            ],
            '\\' => [
                0x7800, 0x7800, 0x3C00, 0x3C00, 0x1E00, 0x1E00, 0x0F00, 0x0F00, 0x0780, 0x0780,
                0x03C0, 0x03C0, 0x01E0, 0x01E0, 0x00F0, 0x00F0, 0x0078, 0x0078, 0x003C, 0x003C,
                0x001E, 0x001E, 0x000F, 0x000F, 0x0007, 0x0000,
            ],
            ']' => [
                0x7FF0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
                0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
                0x00F0, 0x00F0, 0x00F0, 0x7FF0, 0x7FF0, 0x0000,
            ],
            '^' => [
                0x00C0, 0x01C0, 0x01C0, 0x03E0, 0x03E0, 0x07F0, 0x07F0, 0x0778, 0x0F78, 0x0F38,
                0x1E3C, 0x1E3C, 0x3C1E, 0x3C1E, 0x380F, 0x780F, 0x7807, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '_' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000,
            ],
            '`' => [
                0x00F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'a' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0FF8, 0x3FFC, 0x3C7C, 0x003E,
                0x003E, 0x003E, 0x07FE, 0x1FFE, 0x3E3E, 0x7C3E, 0x783E, 0x7C3E, 0x7C7E, 0x3FFF,
                0x1FCF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'b' => [
                0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3DF8, 0x3FFE, 0x3F3E, 0x3E1F,
                0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C1F, 0x3C1E, 0x3F3E, 0x3FFC,
                0x3BF0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'c' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03FE, 0x0FFF, 0x1F87, 0x3E00,
                0x3E00, 0x3C00, 0x7C00, 0x7C00, 0x7C00, 0x3C00, 0x3E00, 0x3E00, 0x1F87, 0x0FFF,
                0x03FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'd' => [
                0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x07FF, 0x1FFF, 0x3E3F, 0x3C1F,
                0x7C1F, 0x7C1F, 0x7C1F, 0x781F, 0x781F, 0x7C1F, 0x7C1F, 0x3C3F, 0x3E7F, 0x1FFF,
                0x0FDF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'e' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03F8, 0x0FFC, 0x1F3E, 0x3E1E,
                0x3C1F, 0x7C1F, 0x7FFF, 0x7FFF, 0x7C00, 0x7C00, 0x3C00, 0x3E00, 0x1F07, 0x0FFF,
                0x03FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'f' => [
                0x01FF, 0x03E1, 0x03C0, 0x07C0, 0x07C0, 0x07C0, 0x7FFF, 0x7FFF, 0x07C0, 0x07C0,
                0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0, 0x07C0,
                0x07C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'g' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07EF, 0x1FFF, 0x3E7F, 0x3C1F,
                0x7C1F, 0x7C1F, 0x781F, 0x781F, 0x781F, 0x7C1F, 0x7C1F, 0x3C3F, 0x3E7F, 0x1FFF,
                0x0FDF, 0x001E, 0x001E, 0x001E, 0x387C, 0x3FF8,
            ],
            'h' => [
                0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3DFC, 0x3FFE, 0x3F9E, 0x3F1F,
                0x3E1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F,
                0x3C1F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'i' => [
                0x01F0, 0x01F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x7FE0, 0x7FE0, 0x01E0, 0x01E0,
                0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0,
                0x01E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'j' => [
                0x00F8, 0x00F8, 0x0000, 0x0000, 0x0000, 0x0000, 0x3FF8, 0x3FF8, 0x00F8, 0x00F8,
                0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8, 0x00F8,
                0x00F8, 0x00F8, 0x00F8, 0x00F0, 0x71F0, 0x7FE0,
            ],
            'k' => [
                0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C1F, 0x3C3E, 0x3C7C, 0x3CF8,
                0x3DF0, 0x3DE0, 0x3FC0, 0x3FC0, 0x3FE0, 0x3DF0, 0x3CF8, 0x3C7C, 0x3C3E, 0x3C1F,
                0x3C1F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'l' => [
                0x7FF0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0,
                0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0, 0x01F0,
                0x01F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'm' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF79E, 0xFFFF, 0xFFFF, 0xFFFF,
                0xFBE7, 0xF9E7, 0xF1C7, 0xF1C7, 0xF1C7, 0xF1C7, 0xF1C7, 0xF1C7, 0xF1C7, 0xF1C7,
                0xF1C7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'n' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3DFC, 0x3FFE, 0x3F9E, 0x3F1F,
                0x3E1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F, 0x3C1F,
                0x3C1F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'o' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07F0, 0x1FFC, 0x3E3E, 0x3C1F,
                0x7C1F, 0x780F, 0x780F, 0x780F, 0x780F, 0x780F, 0x7C1F, 0x3C1F, 0x3E3E, 0x1FFC,
                0x07F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'p' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3DF8, 0x3FFE, 0x3F3E, 0x3E1F,
                0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C1F, 0x3E1E, 0x3F3E, 0x3FFC,
                0x3FF8, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00,
            ],
            'q' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07EE, 0x1FFE, 0x3E7E, 0x3C1E,
                0x7C1E, 0x781E, 0x781E, 0x781E, 0x781E, 0x781E, 0x7C1E, 0x7C3E, 0x3E7E, 0x1FFE,
                0x0FDE, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E,
            ],
            'r' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1F7F, 0x1FFF, 0x1FE7, 0x1FC7,
                0x1F87, 0x1F00, 0x1F00, 0x1F00, 0x1F00, 0x1F00, 0x1F00, 0x1F00, 0x1F00, 0x1F00,
                0x1F00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            's' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07FC, 0x1FFE, 0x1E0E, 0x3E00,
                0x3E00, 0x3F00, 0x1FE0, 0x07FC, 0x00FE, 0x003E, 0x001E, 0x001E, 0x3C3E, 0x3FFC,
                0x1FF0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            't' => [
                0x0000, 0x0000, 0x0000, 0x0780, 0x0780, 0x0780, 0x7FFF, 0x7FFF, 0x0780, 0x0780,
                0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x0780, 0x07C0, 0x03FF,
                0x01FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'u' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3C1E, 0x3C1E, 0x3C1E, 0x3C1E,
                0x3C1E, 0x3C1E, 0x3C1E, 0x3C1E, 0x3C1E, 0x3C1E, 0x3C3E, 0x3C7E, 0x3EFE, 0x1FFE,
                0x0FDE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'v' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF007, 0x780F, 0x780F, 0x3C1E,
                0x3C1E, 0x3E1E, 0x1E3C, 0x1E3C, 0x0F78, 0x0F78, 0x0FF0, 0x07F0, 0x07F0, 0x03E0,
                0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'w' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF003, 0xF1E3, 0xF3E3, 0xF3E7,
                0xF3F7, 0xF3F7, 0x7FF7, 0x7F77, 0x7F7F, 0x7F7F, 0x7F7F, 0x3E3E, 0x3E3E, 0x3E3E,
                0x3E3E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'x' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7C0F, 0x3E1E, 0x3E3C, 0x1F3C,
                0x0FF8, 0x07F0, 0x07F0, 0x03E0, 0x07F0, 0x07F8, 0x0FF8, 0x1E7C, 0x3E3E, 0x3C1F,
                0x781F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            'y' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF807, 0x780F, 0x7C0F, 0x3C1E,
                0x3C1E, 0x1E3C, 0x1E3C, 0x1F3C, 0x0F78, 0x0FF8, 0x07F0, 0x07F0, 0x03E0, 0x03E0,
                0x03C0, 0x03C0, 0x03C0, 0x0780, 0x0F80, 0x7F00,
            ],
            'z' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x001F, 0x003E,
                0x007C, 0x00F8, 0x01F0, 0x03E0, 0x07C0, 0x0F80, 0x1F00, 0x1E00, 0x3C00, 0x7FFF,
                0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            '{' => [
                0x01FE, 0x03E0, 0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x01E0, 0x01E0, 0x01E0, 0x01C0,
                0x03C0, 0x3F80, 0x3F80, 0x03C0, 0x01C0, 0x01E0, 0x01E0, 0x01E0, 0x03C0, 0x03C0,
                0x03C0, 0x03C0, 0x03E0, 0x01FE, 0x007E, 0x0000,
            ],
            '|' => [
                0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0,
                0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0,
                0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x0000,
            ],
            '}' => [
                0x3FC0, 0x03E0, 0x01E0, 0x01E0, 0x01E0, 0x01E0, 0x01C0, 0x03C0, 0x03C0, 0x01C0,
                0x01E0, 0x00FE, 0x00FE, 0x01E0, 0x01C0, 0x03C0, 0x03C0, 0x01C0, 0x01E0, 0x01E0,
                0x01E0, 0x01E0, 0x03E0, 0x3FC0, 0x3F00, 0x0000,
            ],
            '~' => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x3F07, 0x7FC7, 0x73E7, 0xF1FF, 0xF07E, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
            _ => [
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            ],
        }
    }
}
