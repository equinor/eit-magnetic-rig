mod utils;

use wasm_bindgen::prelude::*;


#[wasm_bindgen]
pub fn crc16(msg: &[u8]) -> u16
{
    let mut crc: u16 = 0xffff;
    for byte in msg {
        crc ^= *byte as u16;
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xa001;
            } else { 
                crc >>= 1;
            }
        }
    }

    return crc;
}
