#!/usr/bin/env python3
import unittest
import random
import struct
import time
import statistics

import usb
from usb.core import Device

class BluepillFixture:
    def __init__(self, serial, vid=0x1209, pid=0x6827):
        self.vid, self.pid, self.serial = vid, pid, f'bp1-{serial:04d}' if serial else None
        self._connect()

    def write(self, pattern=None, seed=None):
        self._write_op(pattern, seed, op=0xd0)

    def xor(self, pattern=None, seed=None):
        self._write_op(pattern, seed, op=0xf0)

    def read(self):
        out = b''
        self._retry(Device.ctrl_transfer, bmRequestType=0x21, bRequest=0x01, wValue=0, wIndex=0)
        while True:
            data = bytes(self._retry(Device.read, endpoint=0x82, size_or_buffer=8192, timeout=100))
            if not data:
                return out

            out += data
            if len(out) > 2e6:
                raise NotImplementedError(f'This code is really not meant to handle more than a few hundred KB of data, got {len(out)/1e6:.0f}MB so far. Aborting.')

    def power_cycle(self, delay_ms, wait=False, timeout=10.0, interval=0.1):
        self._retry(Device.ctrl_transfer, bmRequestType=0x21, bRequest=0x03, wValue=round(delay_ms*100), wIndex=0)
        if wait:
            for _ in range(int(timeout/interval)):
                try:
                    self._dev = usb.core.find(idVendor=self.vid, idProduct=self.pid, **dict(serial_number=self.serial) if self.serial else {})
                    self._dev.set_configuration(1)
                    self._retry(Device.ctrl_transfer, bmRequestType=0x21, bRequest=0x01, wValue=0, wIndex=0) # read
                    break
                except:
                    time.sleep(interval)
            else:
                raise OSError('Timeout while reconnecting to bluepill fixture')

    def read_temperature(self):
        data = self._retry(Device.ctrl_transfer, bmRequestType=0xA1, bRequest=0x02, wValue=0, wIndex=0, data_or_wLength=64)
        temp, = struct.unpack('<h', bytes(data))
        return temp/10 # Device returns signed integer tenths of a degree

    def _write_op(self, pattern, seed, op):
        if pattern is not None:
            self._retry(Device.ctrl_transfer, bmRequestType=0x21, bRequest=op | 1, wValue=0, wIndex=0, data_or_wLength=pattern)
        elif seed is not None:
            self._retry(Device.ctrl_transfer, bmRequestType=0x21, bRequest=op | 2, wValue=0, wIndex=0, data_or_wLength=seed)
        else:
            raise ValueError('Either pattern or seed must be given.')

    def _connect(self):
        self._dev = usb.core.find(idVendor=self.vid, idProduct=self.pid, **dict(serial_number=self.serial) if self.serial else {})
        if self._dev is None:
            raise OSError(f'Device with serial {self.serial} not found')
        self._dev.set_configuration(1)

    def _retry(self, op, *args, **kwargs):
        try:
            print(f'Running {op} with {args}, {kwargs}')
            return op(self._dev, *args, **kwargs)

        except usb.core.USBError as e:
            if e.errno != 19: # No such device/device disconnected
                raise e

            self._connect()
            return op(self._dev, *args, **kwargs)

        except AttributeError as e: # PyUSB is a little buggy and sometimes throws these instead of the above USBError
            self._connect()
            return op(self._dev, *args, **kwargs)

class BluepillFixtureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fix = BluepillFixture(serial=None)
        cls.n = len(cls.fix.read())

    def testBufferLen(self):
        self.assertTrue(BluepillFixtureTest.n >= 512)
        self.assertTrue(BluepillFixtureTest.n%64 == 0)

    def testSingleBytePatternWrite(self):
        for b in bytes([0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x03, 0x06, 0xc0, 0x18, 0x30, 0x60, 0xc0, 0x81, 0x3c, 0x0f, 0xf0, 0xff]):
            pattern = bytes([b])
            BluepillFixtureTest.fix.write(pattern)
            data = BluepillFixtureTest.fix.read()
            self.assertEqual(data, pattern*self.n)

    def testMultiBytePatternWrite(self):
        st = random.Random(0) # Get deterministic pseudorandom output
        for pattern_len in range(2, 64):
            for i in range(8):
                pattern = bytes(st.choices(range(256), k=pattern_len))
                BluepillFixtureTest.fix.write(pattern)
                data = BluepillFixtureTest.fix.read()
                self.assertEqual(data, (pattern*BluepillFixtureTest.n)[:BluepillFixtureTest.n])

    def testPatternXor(self):
        st = random.Random(0) # Get deterministic pseudorandom output
        for pattern_len in range(1, 64):
            for i in range(4):
                pattern = bytes(st.choices(range(256), k=pattern_len))
                BluepillFixtureTest.fix.write(pattern)
                self.assertEqual(BluepillFixtureTest.fix.read(), (pattern*BluepillFixtureTest.n)[:BluepillFixtureTest.n])
                BluepillFixtureTest.fix.xor(pattern)
                self.assertEqual(BluepillFixtureTest.fix.read(), b'\0'*BluepillFixtureTest.n)
                BluepillFixtureTest.fix.xor(pattern)
                self.assertEqual(BluepillFixtureTest.fix.read(), (pattern*BluepillFixtureTest.n)[:BluepillFixtureTest.n])

    def testRandomWrite(self):
        st = random.Random(0) # Get deterministic pseudorandom output
        for _ in range(64):
            BluepillFixtureTest.fix.write(seed=bytes(st.choices(range(256), k=16)))

            data0 = BluepillFixtureTest.fix.read()
            self.assertIn(len(set(data0)), range(128, 257)) # Simple entropy test
            self.assertEqual(len(data0), BluepillFixtureTest.n)

            data1 = BluepillFixtureTest.fix.read()
            self.assertEqual(data0, data1)

            BluepillFixtureTest.fix.write(seed=bytes(st.choices(range(256), k=16)))

            data2 = BluepillFixtureTest.fix.read()
            self.assertNotEqual(data0, data2)
            self.assertEqual(len(data2), BluepillFixtureTest.n)
            self.assertIn(len(set(data2)), range(128, 257))

    def testRandomXor(self):
        st = random.Random(0) # Get deterministic pseudorandom output
        for _ in range(64):
            seed0, seed1 = [bytes(st.choices(range(256), k=16)) for _ in [0, 1]]
            BluepillFixtureTest.fix.write(seed=seed0)

            data0 = BluepillFixtureTest.fix.read()
            self.assertIn(len(set(data0)), range(128, 257))

            BluepillFixtureTest.fix.xor(seed=seed0)
            data1 = BluepillFixtureTest.fix.read()
            self.assertEqual(data1, b'\0'*BluepillFixtureTest.n)

            BluepillFixtureTest.fix.xor(seed=seed0)
            data2 = BluepillFixtureTest.fix.read()
            self.assertEqual(data0, data2)

            BluepillFixtureTest.fix.xor(seed=seed1)
            data3 = BluepillFixtureTest.fix.read()
            self.assertIn(len(set(data3)), range(128, 257))

            BluepillFixtureTest.fix.xor(seed=seed0)
            data4 = BluepillFixtureTest.fix.read()
            self.assertIn(len(set(data4)), range(128, 257))

            BluepillFixtureTest.fix.xor(seed=seed1)
            data5 = BluepillFixtureTest.fix.read()
            self.assertEqual(data5, b'\0'*BluepillFixtureTest.n)

    def testPowerCycle(self):
        BluepillFixtureTest.fix.power_cycle(delay_ms=1000)
        time.sleep(1.0)
        with self.assertRaises(OSError):
            BluepillFixtureTest.fix.power_cycle(delay_ms=100)
        time.sleep(1.5)
        BluepillFixtureTest.fix.power_cycle(delay_ms=100)
        time.sleep(2.0) # Wait to make sure the device is there again for the next test

    def testReadTemperature(self):
        temps = []
        for _ in range(1000):
            temp = BluepillFixtureTest.fix.read_temperature()
            self.assertEqual(type(temp), float)
            self.assertTrue(10 <= temp <= 50) # device temp may be higher than ambient temp
            temps.append(temp)
        self.assertTrue(10 <= statistics.mean(temps) <= 50)
        self.assertTrue(-5 <= statistics.stdev(temps) <= 5)

