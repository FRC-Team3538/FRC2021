use std::{env, fs::File, io::Read};

use wpilog_reader::{parse_wpilog, reorganize};

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut file = File::open(&args[1]).unwrap();

    let mut content = Vec::new();
    file.read_to_end(&mut content).unwrap();

    let parsed_log = parse_wpilog(&content).unwrap().1;
    let organized_log = reorganize(parsed_log);

    println!("{:?}", organized_log);
}
