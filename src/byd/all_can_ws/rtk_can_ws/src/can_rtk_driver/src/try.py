# 你的 8 字节十六进制（小端序）
hex_bytes = "A6 41 3C C9 05 00 00 00"

# 1. 去掉空格，转为字节串
data = bytes.fromhex(hex_bytes.replace(" ", ""))

# 2. 小端解析为 有符号 int64
decimal_value = int.from_bytes(data, byteorder="little", signed=True)

# 3. 输出结果
print(f"十六进制 (小端): {hex_bytes}")
print(f"十进制 int64: {decimal_value}")
