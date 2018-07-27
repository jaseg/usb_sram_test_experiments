
import modes
import binascii
import hashlib

class Pattern:
    def __init__(self, param=None):
        self.spec = f'pattern:{param}'
        self.param = binascii.unhexlify(param.encode())

    def write(self, fix):
        fix.write(pattern=self.param)

    def xor(self, fix):
        fix.xor(pattern=self.param)

class Random:
    def __init__(self, param=None):
        self.spec = f'pattern:{param}'
        self.param = hashlib.sha256(param.encode()).digest()[:16]

    def write(self, fix):
        fix.write(seed=self.param)

    def xor(self, fix):
        fix.xor(seed=self.param)

MODES = {
        'pattern': Pattern,
        'random': Random
        }
