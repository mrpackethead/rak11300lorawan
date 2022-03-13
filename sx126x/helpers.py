from utime import sleep_ms

def ASSERT(state):
    assert state == ERR_NONE, ERROR[state]

def yield_():
    sleep_ms(1)