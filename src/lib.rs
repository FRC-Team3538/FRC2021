use parser::{
    parse_array, parse_array_ref, parse_boolean, parse_double, parse_float, parse_int64, parse_raw,
    parse_string,
};
use types::{ControlRecord, DataEntry, DataPoint, DataType, OrganizedLog, Record, WpiLog};

mod parser;
mod types;

pub use parser::parse_wpilog;

pub fn reorganize(input: WpiLog) -> OrganizedLog {
    let mut data = Vec::new();

    let mut unfinished_data = Vec::new();

    for record in input.records {
        match record.data {
            Record::Data(data) => {
                unfinished_data
                    .iter_mut()
                    .filter(|rec: &&mut DataEntry| rec.entry_id == record.entry_id)
                    .for_each(|rec| {
                        let data = match rec.typ {
                            "boolean" => DataType::Boolean(parse_boolean(data.data).unwrap().1),
                            "int64" => DataType::Int64(parse_int64(data.data).unwrap().1),
                            "float" => DataType::Float(parse_float(data.data).unwrap().1),
                            "double" => DataType::Double(parse_double(data.data).unwrap().1),
                            "string" => DataType::String(parse_string(data.data).unwrap().1),
                            "boolean[]" => DataType::BooleanArray(
                                parse_array(parse_boolean, data.data).unwrap().1,
                            ),
                            "int64[]" => {
                                DataType::Int64Array(parse_array(parse_int64, data.data).unwrap().1)
                            }
                            "float[]" => {
                                DataType::FloatArray(parse_array(parse_float, data.data).unwrap().1)
                            }
                            "double[]" => DataType::DoubleArray(
                                parse_array(parse_double, data.data).unwrap().1,
                            ),
                            "string[]" => DataType::StringArray(
                                parse_array_ref(parse_string, data.data).unwrap().1,
                            ),
                            _ => DataType::Raw(parse_raw(data.data).unwrap().1),
                        };
                        rec.data.push(DataPoint {
                            timestamp: record.timestamp_us,
                            data,
                        });
                    });
            }
            Record::Control(control) => match control {
                ControlRecord::Start(start) => {
                    let entry = DataEntry {
                        entry_id: start.entry_id,
                        name: start.name,
                        typ: start.typ,
                        metadata: start.metadata,
                        data: Vec::new(),
                    };

                    unfinished_data.push(entry);
                }
                ControlRecord::Finish(finish) => {
                    let unf_ind = unfinished_data
                        .iter()
                        .enumerate()
                        .find(|rec| rec.1.entry_id == finish.entry_id)
                        .map(|rec| rec.0)
                        .unwrap();

                    let finished_data = unfinished_data.remove(unf_ind);
                    data.push(finished_data);
                }
                ControlRecord::SetMetadata(set_metadata) => {
                    unfinished_data
                        .iter_mut()
                        .filter(|rec| rec.entry_id == set_metadata.entry_id)
                        .for_each(|rec| rec.metadata = set_metadata.metadata);
                }
            },
        }
    }

    data.append(&mut unfinished_data);

    OrganizedLog {
        sessioned_data: data,
    }
}
