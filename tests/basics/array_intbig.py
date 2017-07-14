# test array types QqLl that require big-ints

try:
    from array import array
except ImportError:
    print("SKIP")
    raise SystemExit

print(array('L', [0, 2**32-1]))
print(array('l', [-2**31, 0, 2**31-1]))

# Skip if long ints are not supported.
try:
    # We have to use variables because 1 << 40 causes an exception on parse and
    # cannot be caught.
    x = 40
    x = 1 << x
except OverflowError:
    print("SKIP")
    import sys
    sys.exit()

print(array('q'))
print(array('Q'))

print(array('q', [0]))
print(array('Q', [0]))

print(array('q', [-2**63, -1, 0, 1, 2, 2**63-1]))
print(array('Q', [0, 1, 2, 2**64-1]))

print(bytes(array('q', [-1])))
print(bytes(array('Q', [2**64-1])))
