
    pub const fn make_prefix(device_type: u8, device_id: u8) -> [u8;3] {
        [2, device_type, device_id + 48]
    }
    
    pub fn int_to_bytes<T: ToString>(number: T) -> Vec<u8> {
        number.to_string()
            .chars()
            .map(|c| c as u8)
            .collect()
    }
    
    pub fn int_to_byte(number: u8) -> u8 {
        number + 48
    }
    
    pub fn bytes_to_int(bytes: &[u8]) -> isize {
        let sign = if bytes[0] == 45 {-1} else {1};
        let int = bytes.iter()
            .filter(|&&x| (48..=57).contains(&x))
            .fold(0, |mut acc, x| {
                let num = x - 48;
                acc*=10;
                acc+= num as isize;
                acc
            });
        int * sign
    }
    
   
#[cfg(test)]
mod tests {
    use crate::controls_components::helper::{bytes_to_int, int_to_bytes, make_prefix};
    #[test]
    fn test_make_prefix() {
        let prefix = make_prefix(77, 2);
        assert_eq!(prefix, [2, 77, 50]);
    }
    
    #[test]
    fn test_int_to_bytes() {
        let bytes = int_to_bytes(2300);
        assert_eq!(bytes, [50,51,48,48]);
        let bytes = int_to_bytes(-3400);
        assert_eq!(bytes, [45,51,52,48,48]);
    }
    
    #[test]
    fn test_bytes_to_int() {
        let int = bytes_to_int([45, 51, 52, 48, 48,13].as_slice());
        assert_eq!(-3400, int);
        let int = bytes_to_int([50,51,48,48].as_slice());
        assert_eq!(2300, int);
    }
}