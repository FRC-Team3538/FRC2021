use std::{env, fs::File, io::Read, path::Path};

use wpilog_reader::{parse_wpilog, reorganize, sort, to_array};

use wpilog_reader::types::DataType;

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut infile = File::open(&args[1]).unwrap();

    let mut content = Vec::new();
    infile.read_to_end(&mut content).unwrap();

    let parsed_log = parse_wpilog(&content).unwrap().1;
    let mut organized_log = reorganize(parsed_log);

    sort(&mut organized_log);
    let csv_log = to_array(&organized_log);

    let outfile = Path::new(&args[2]);

    let mut csvwriter = csv::Writer::from_path(outfile).unwrap();

    csvwriter.write_field("timestamp").unwrap();
    for field in &organized_log.sessioned_data {
        csvwriter.write_field(field.name).unwrap();
    }
    csvwriter.write_record(None::<&[u8]>).unwrap();

    // csvwriter.write_field("int64").unwrap();
    // for field in &organized_log.sessioned_data {
    //     csvwriter.write_field(field.typ).unwrap();
    // }
    // csvwriter.write_record(None::<&[u8]>).unwrap();

    // csvwriter.write_field(&[]).unwrap();
    // for field in &organized_log.sessioned_data {
    //     csvwriter.write_field(field.metadata).unwrap();
    // }
    // csvwriter.write_record(None::<&[u8]>).unwrap();

    for row in csv_log {
        for field in row {
            match field {
                Some(DataType::Raw(raw)) => csvwriter.write_field(format!("{:X?}", raw)).unwrap(),
                Some(DataType::Boolean(boolean)) => csvwriter
                    .write_field(format!("{:X?}", boolean as u8))
                    .unwrap(),
                Some(DataType::Int64(int64)) => {
                    csvwriter.write_field(format!("{}", int64)).unwrap()
                }
                Some(DataType::Float(float)) => {
                    csvwriter.write_field(format!("{}", float)).unwrap()
                }
                Some(DataType::Double(double)) => {
                    csvwriter.write_field(format!("{}", double)).unwrap()
                }
                _ => csvwriter.write_field(&[]).unwrap(),
                // Some(DataType::String(string)) => csvwriter.write_field(string).unwrap(),
                // Some(DataType::BooleanArray(booleanArray)) => {}
                // Some(DataType::Int64Array(int64Array)) => {}
                // Some(DataType::FloatArray(floatArray)) => {}
                // Some(DataType::DoubleArray(doubleArray)) => {}
                // Some(DataType::StringArray(stringArray)) => {},
                // None => csvwriter.write_field("0").unwrap(),
            }
        }
        csvwriter.write_record(None::<&[u8]>).unwrap();
        csvwriter.flush().unwrap();
    }

    csvwriter.flush().unwrap();
}
