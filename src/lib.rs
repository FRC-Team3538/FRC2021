use parser::{
    parse_array, parse_array_ref_with_len, parse_boolean, parse_double, parse_float, parse_int64,
    parse_raw, parse_string_full, parse_string_with_len,
};
use types::{ControlRecord, DataEntry, DataPoint, DataType, OrganizedLog, Record, WpiLog};

mod parser;
pub mod types;

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
                            "string" => DataType::String(parse_string_full(data.data).unwrap().1),
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
                                parse_array_ref_with_len(parse_string_with_len, data.data)
                                    .unwrap()
                                    .1,
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

pub fn sort(log: &mut OrganizedLog) {
    for record in &mut log.sessioned_data {
        record.data.sort_by_cached_key(|point| point.timestamp);
    }
}

// the log _should_ be sorted, but it can function without
pub fn to_array<'a>(log: &'a OrganizedLog) -> Vec<Vec<Option<DataType<'a>>>> {
    let entry_count = log.sessioned_data.len();

    let mut next_index = vec![0; entry_count];

    let mut records = Vec::new();

    loop {
        let mut record = vec![None; entry_count + 1];

        if let Some((index, entry)) =
            log.sessioned_data
                .iter()
                .enumerate()
                .min_by_key(|(ind, entry)| {
                    if entry.data.is_empty() || next_index[*ind] == entry.data.len() {
                        u64::MAX
                    } else {
                        entry.data[next_index[*ind]].timestamp
                    }
                })
        {
            if entry.data.is_empty() || next_index[index] == entry.data.len() {
                break;
            }

            record[0] = Some(DataType::Double(
                entry.data[next_index[index]].timestamp as f64 / 1000000.0,
            ));

            record[index + 1] = Some(entry.data[next_index[index]].data.clone());
            next_index[index] += 1;
        } else {
            break;
        }

        records.push(record);
    }

    records
}
