# Random tests
import math

if 1:
    def isqrt(x):
        op = x
        res = 0
        one = 1 << 30

        while one > op:
            one >>= 2
        while one != 0:
            if op >= res + one:
                op -= res + one
                res += one << 1
            res >>= 1
            one >>= 2
        return res

    def isqrt64(x):
        op = x
        res = 0
        one = 1 << 62

        while one > op:
            one >>= 2
        while one != 0:
            if op >= res + one:
                op -= res + one
                res += one << 1
            res >>= 1
            one >>= 2
        return res

    for x in range(0, 2 ** 64, 2 ** 48):
        i = isqrt64(x)
        s = int(math.sqrt(x))
        if i != s:
            print(x, i, s)
    print("finish")
