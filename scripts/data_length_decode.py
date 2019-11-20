data_lengths = [
  ("0x00", "00"),
  ("0x01", "01"),
  ("0x02", "02"),
  ("0x03", "03"),
  ("0x04", "04"),
  ("0x05", "05"),
  ("0x06", "06"),
  ("0x07", "07"),
  ("0x08", "08"),
  ("0x09", "12"),
  ("0x0a", "16"),
  ("0x0b", "20"),
  ("0x0c", "24"),
  ("0x0d", "32"),
  ("0x0e", "48"),
  ("0x0f", "64")
]

for (code, length) in data_lengths:
  print("case {}:".format(code))
  print("    *p_data_length = MCP_DATA_LENGTH_{}_BYTES;".format(length))
  print("    break;")
  print()
