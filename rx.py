from sx1262 import SX1262
import time
import radio_cfg.radio_cfg as radio

sx = SX1262()
sx.begin(**radio.AS923)

while True:
    msg, err = sx.recv()
    if len(msg) > 0:
        error = SX1262.STATUS[err]
        print(msg)
        print(error)