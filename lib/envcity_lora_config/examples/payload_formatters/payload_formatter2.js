function decodeUplink(input) {
    
    var decoded_data = {}
    var data = [];
    for(var i = 1; i < input.bytes.length; i+=2){
        
        //uint8_t b = (aux & 0xF000) >> 12;
        //uint16_t f = aux & 0x0FFF; 
        var b = (input.bytes[i] & 0xF0) >> 4;
        var f =  ((input.bytes[i] & 0x0F) << 8) + input.bytes[i-1];
        // f/4096 * 2^(b-15)
        data.push(f / 4096.0 * Math.pow(2, b - 15) * 6.144); // 6.144 max voltage adc
        
    }
    
    [decoded_data.co, decoded_data.h2s, decoded_data.nh3, decoded_data.so2, decoded_data.no2, decoded_data.ox, 
    decoded_data.temp, decoded_data.humidity, decoded_data.wind_speed] =  data
    
    return {
        data: decoded_data,
        warnings: [],
        errors: []
      };
}
