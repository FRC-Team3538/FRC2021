#[derive(Default, Debug, Clone)]
pub struct WpiLog<'a> {
    pub major_version: u8,
    pub minor_version: u8,
    pub records: Vec<WpiRecord<'a>>,
}

#[derive(Debug, Clone)]
pub struct WpiRecord<'a> {
    pub entry_id: u32,
    pub timestamp_us: u64,
    pub data: Record<'a>,
}

#[derive(Debug, Clone)]
pub enum Record<'a> {
    Data(DataRecord<'a>),
    Control(ControlRecord<'a>),
}

#[derive(Default, Debug, Clone)]
pub struct DataRecord<'a> {
    pub data: &'a [u8],
}

#[derive(Debug, Clone)]
pub enum ControlRecord<'a> {
    Start(StartRecord<'a>),
    Finish(FinishRecord),
    SetMetadata(SetMetadataRecord<'a>),
}

#[derive(Default, Debug, Clone)]
pub struct StartRecord<'a> {
    pub entry_id: u32,
    pub name: &'a str,
    pub typ: &'a str,
    pub metadata: &'a str,
}

#[derive(Default, Debug, Clone)]
pub struct FinishRecord {
    pub entry_id: u32,
}

#[derive(Default, Debug, Clone)]
pub struct SetMetadataRecord<'a> {
    pub entry_id: u32,
    pub metadata: &'a str,
}

#[derive(Default, Debug, Clone)]
pub struct OrganizedLog<'a> {
    pub sessioned_data: Vec<DataEntry<'a>>,
}

#[derive(Default, Debug, Clone)]
pub struct DataEntry<'a> {
    pub entry_id: u32,
    pub name: &'a str,
    pub typ: &'a str,
    pub metadata: &'a str,
    pub data: Vec<DataPoint<'a>>,
}

#[derive(Debug, Clone)]
pub struct DataPoint<'a> {
    pub timestamp: u64,
    pub data: DataType<'a>,
}

#[derive(Debug, Clone)]
pub enum DataType<'a> {
    Raw(&'a [u8]),
    Boolean(bool),
    Int64(i64),
    Float(f32),
    Double(f64),
    String(&'a str),
    BooleanArray(Vec<bool>),
    Int64Array(Vec<i64>),
    FloatArray(Vec<f32>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<&'a str>),
}
