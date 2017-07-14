# Skip this test if reversed() isn't built in.
import builtins
if "reversed" not in dir(builtins):
    print("SKIP")
    import sys
    sys.exit()

# argument to fromkeys has no __len__
d = dict.fromkeys(reversed(range(1)))
print(d)
