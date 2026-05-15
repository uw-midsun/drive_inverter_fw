use crate::comms_schema;

#[derive(Clone, Debug)]
pub struct CanConfig {
    pub iface: String,
    pub base_mc: u32,
    pub base_dc: u32,
}

impl Default for CanConfig {
    fn default() -> Self {
        Self {
            iface: "can0".to_string(),
            base_mc: comms_schema::BASE_MC,
            base_dc: comms_schema::BASE_DC,
        }
    }
}

pub(crate) fn parse_hex(s: &str) -> Option<u32> {
    let s = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(s, 16).ok()
}

impl CanConfig {
    pub fn parse(iface: &str, mc: &str, dc: &str) -> Result<Self, String> {
        let iface = iface.trim();
        if iface.is_empty() {
            return Err("Interface name cannot be empty".into());
        }
        let base_mc = parse_hex(mc).ok_or_else(|| format!("Invalid MC base: {}", mc))?;
        let base_dc = parse_hex(dc).ok_or_else(|| format!("Invalid DC base: {}", dc))?;
        if base_mc == base_dc {
            return Err("MC and DC base addresses must differ".into());
        }
        Ok(Self {
            iface: iface.to_string(),
            base_mc,
            base_dc,
        })
    }
}
