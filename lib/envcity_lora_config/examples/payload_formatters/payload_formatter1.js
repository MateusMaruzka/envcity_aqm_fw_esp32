function decodeUplink(input) {
    var str = {}
    str = String.fromCharCode.apply(null, input.bytes);
    str = str.split(",")
    return {
      data: {
        field1: parseInt(str[0])/1000,
        field2: parseInt(str[1])/1000,
        field3: parseInt(str[2])/1000,
        field4: parseInt(str[3])/1000,
        field5: parseInt(str[4])/1000,
        field6: parseInt(str[5])/1000,
        field7: parseInt(str[6])/1000,
        field8: parseInt(str[7])/1000
      },
      warnings: [],
      errors: []
    };
  }