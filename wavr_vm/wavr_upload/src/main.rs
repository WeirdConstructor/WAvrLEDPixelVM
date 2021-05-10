use serial::*;
use std::io::Read;

fn main() {
    let args : Vec<String> = std::env::args().collect();
    let port = &args[1];
    let mut port = open(port).unwrap();

    port.reconfigure(&|settings| {
        settings.set_baud_rate(serial::Baud9600)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    }).unwrap();

    let mut buf = [0; 200];

    port.set_timeout(std::time::Duration::from_millis(2000)).unwrap();

    while let Ok(len) = port.read(&mut buf) {
        let s = String::from_utf8_lossy(&buf[0..len]);
        println!("> {}", s);
    }
}
