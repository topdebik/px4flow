def shift32(num):
    if num > 2147483648:
        return -4294967296 + num
    return num

def shift16(num):
    if num > 32768:
        return -65536 + num
    return num

def shift8(num):
    if num > 128:
        return -256 + num
    return num