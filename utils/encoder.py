def encode_float_to_bytes(value, min_value, max_value, byte_count=4):
    """
    float → 整数スケーリング → バイトリスト
    """
    if not min_value <= value <= max_value:
        raise ValueError("値が範囲外です")
    if not 1 <= byte_count <= 4:
        raise ValueError("byte_countは1〜4の範囲で指定してください")
 
    bit_len = byte_count * 8
    max_int = (1 << (bit_len - 1)) - 1  # 例: 0x7FFF, 0x7FFFFF, 0x7FFFFFFF
    norm = (value - min_value) / (max_value - min_value)
    scaled = int((norm * 2 - 1) * max_int)
 
    if scaled < 0:
        scaled += 1 << bit_len  # 2の補数に変換
 
    return [(scaled >> (8 * i)) & 0xFF for i in reversed(range(byte_count))]
 
 
def decode_bytes_to_float(byte_list, min_value, max_value):
    """
    バイトリスト → 整数復元 → float変換
    """
    byte_count = len(byte_list)
    bit_len = byte_count * 8
    max_int = (1 << (bit_len - 1)) - 1
 
    value = sum(byte_list[i] << (8 * (byte_count - 1 - i)) for i in range(byte_count))
 
    if value & (1 << (bit_len - 1)):
        value -= 1 << bit_len  # 2の補数 → 符号付き整数に戻す
 
    norm = value / max_int
    return ((norm + 1) / 2) * (max_value - min_value) + min_value

def send_float_over_i2c(bus, addr, cmd, value, min_value, max_value, byte_count=4):
    bit_len = byte_count * 8
    max_int = (1 << (bit_len - 1)) - 1
    norm = (value - min_value) / (max_value - min_value)
    scaled = int((norm * 2 - 1) * max_int)
    if scaled < 0:
        scaled += 1 << bit_len
    data = [(scaled >> (8 * i)) & 0xFF for i in reversed(range(byte_count))]
    bus.write_i2c_block_data(addr, cmd, data)

def send_float_over_i2c(bus, addr, cmd, value, min_value, max_value, byte_count=4):
    """
    floatをスケーリングしてI2C経由で送信
    """
    byte_list = encode_float_to_bytes(value, min_value, max_value, byte_count)
    bus.write_i2c_block_data(addr, cmd, byte_list)