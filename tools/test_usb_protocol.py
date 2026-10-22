#!/usr/bin/env python3
"""验证实际 USB 收发函数的分包解析、应答格式和发送通知。"""

import ctypes as ct
from pathlib import Path
import random
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)


def function(source, name):
    match = re.search(r"(?m)^(?:static )?[\w *]+\b" + name + r"\([^;]*?\)\n\{", source)
    if match is None:
        raise AssertionError(f"找不到函数 {name}")
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


class Message(ct.Structure):
    _fields_ = [("id", ct.c_uint16), ("length", ct.c_uint8),
                ("data", ct.c_uint8 * 64)]


class UsbProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="usb-protocol-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        usb = (ROOT / "Comm/USB_Thread.c").read_text()
        protocol = (ROOT / "Comm/Protocol.c").read_text()
        # 编译实际函数，只替代线程通知和接收端，避免复制被测逻辑。
        source = '''
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "Protocol.h"
typedef uint32_t ULONG;
typedef unsigned UINT;
typedef struct { int Unused; } UX_SLAVE_CLASS_CDC_ACM;
#define UX_NULL NULL
#define UX_SUCCESS 0U
#define TX_OR 1U
#define AXDR_CANFD_INVALID_LEN 0xFFU
static ULONG USB_Tx_Event, USB_Tx_Pending;
static UX_SLAVE_CLASS_CDC_ACM Test_Cdc;
static UX_SLAVE_CLASS_CDC_ACM *volatile Cdc_Acm;
static jmp_buf Test_Rx_End;
static unsigned Test_Rx_Mode, Test_Reads;
uint32_t Test_Disconnect_On_Rx;
void USB_Deactivate(void);
uint32_t Test_Isr, Test_Primask, Test_Wakes, Test_Count;
AxDr_Msg_T Test_Messages[512], Test_Response;
static uint32_t __get_IPSR(void) { return Test_Isr; }
static uint32_t __get_PRIMASK(void) { return Test_Primask; }
static void __disable_irq(void) { Test_Primask = 1; }
static void __set_PRIMASK(uint32_t Value) { Test_Primask = Value; }
static int tx_event_flags_set(ULONG *Event, ULONG Flag, unsigned Op)
{
    assert(Op == TX_OR);
    *Event |= Flag;
    Test_Wakes++;
    return 0;
}
static bool Response_Push(const AxDr_Msg_T *Msg)
{
    Test_Response = *Msg;
    return true;
}
void Protocol_Rx(uint16_t Id, const uint8_t *Data, uint8_t Len)
{
    assert(Test_Count < 512);
    Test_Messages[Test_Count].Id = Id;
    Test_Messages[Test_Count].Len = Len;
    memcpy(Test_Messages[Test_Count++].Data, Data, Len);
    if (Test_Disconnect_On_Rx)
    {
        Test_Disconnect_On_Rx = 0;
        USB_Deactivate();
    }
}
static void tx_thread_sleep(ULONG Ticks)
{
    assert(Ticks == 1 || Ticks == 10);
    Cdc_Acm = &Test_Cdc;
}
static UINT ux_device_class_cdc_acm_read(UX_SLAVE_CLASS_CDC_ACM *Cdc,
                                       uint8_t *Buf, ULONG Size, ULONG *Len)
{
    const uint8_t Frame[] = {'A', 'X', 'D', 'R', 1, 0, 1, 42};
    assert(Cdc == &Test_Cdc && Size >= sizeof(Frame));
    if (Test_Reads++ == 0 && Test_Rx_Mode != 0)
    {
        if (Test_Rx_Mode == 1)
        {
            USB_Deactivate();
        }
        return 1;
    }
    if (Test_Reads > (Test_Rx_Mode == 0 ? 1U : 2U))
    {
        longjmp(Test_Rx_End, 1);
    }
    memcpy(Buf, Frame, sizeof(Frame));
    *Len = sizeof(Frame);
    return UX_SUCCESS;
}
'''
        source += "\n".join(line for line in usb.splitlines() if line.startswith("#define USB_"))
        source += '''
static uint8_t USB_Rx_Stream[USB_RX_STREAM_SIZE];
static uint16_t USB_Rx_Len;
'''
        for name in ("AxDr_CANFD_Length", "AxDr_CANFD_Length_Valid", "Response",
                     "Parameter_Status_Map", "Protocol_Parameter_Write_Response",
                     "Protocol_Write_Response"):
            source += function(protocol, name) + "\n"
        for name in ("USB_Frame_Pack", "USB_Rx_Data", "USB_Deactivate",
                     "USB_Rx_Thread", "USB_Tx_Wake", "USB_Tx_Poll"):
            source += function(usb, name) + "\n"
        source += '''
void Test_Reset(void)
{
    USB_Rx_Len = Test_Count = Test_Wakes = Test_Isr = Test_Primask = 0;
    USB_Tx_Event = USB_Tx_Pending = 0;
    Test_Disconnect_On_Rx = 0;
    Cdc_Acm = &Test_Cdc;
}
void Test_Feed(const uint8_t *Data, uint16_t Len)
{
    assert(Len <= USB_RX_READ_SIZE);
    USB_Rx_Data(Data, Len);
    assert(USB_Rx_Len < USB_RX_STREAM_SIZE);
}
void Test_Run_Rx(unsigned Mode)
{
    Test_Rx_Mode = Mode;
    Test_Reads = 0;
    if (Mode == 0)
    {
        USB_Deactivate();
    }
    if (setjmp(Test_Rx_End) == 0)
    {
        USB_Rx_Thread();
    }
}
uint16_t Test_Pack(uint8_t *Buf, const AxDr_Msg_T *Msg)
{
    return USB_Frame_Pack(Buf, Msg);
}
'''
        (path / "test.c").write_text(source)
        library = path / "test.so"
        subprocess.run(["cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-Wl,--no-undefined", "-IComm", "-IParameter", "-IMotor",
                        str(path / "test.c"), "-o", str(library)], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Test_Feed.argtypes = [ct.c_char_p, ct.c_uint16]
        cls.lib.Test_Pack.argtypes = [ct.c_void_p, ct.POINTER(Message)]
        cls.lib.Test_Pack.restype = ct.c_uint16
        cls.lib.AxDr_CANFD_Length_Valid.argtypes = [ct.c_uint8]
        cls.lib.AxDr_CANFD_Length_Valid.restype = ct.c_bool

    def setUp(self):
        self.lib.Test_Reset()

    def feed(self, data):
        for start in range(0, len(data), 64):
            chunk = data[start:start + 64]
            self.lib.Test_Feed(chunk, len(chunk))

    def received(self):
        count = ct.c_uint32.in_dll(self.lib, "Test_Count").value
        messages = (Message * 512).in_dll(self.lib, "Test_Messages")
        return [(m.id, bytes(m.data[:m.length])) for m in messages[:count]]

    def test_all_frame_lengths_and_split_positions(self):
        for length in LENGTHS:
            payload = bytes(range(length))
            wire = b"AXDR\xc1\x01" + bytes([length]) + payload
            msg = Message(0x1C1, length, (ct.c_uint8 * 64)(*payload))
            output = ct.create_string_buffer(71)
            self.assertEqual(self.lib.Test_Pack(output, ct.byref(msg)), len(wire))
            self.assertEqual(output.raw[:len(wire)], wire)
            for split in range(len(wire) + 1):
                self.lib.Test_Reset()
                self.feed(wire[:split])
                self.feed(wire[split:])
                self.assertEqual(self.received(), [(msg.id, payload)])

    def test_joined_frames_with_random_read_boundaries(self):
        rng = random.Random(17)
        expected = [(rng.randrange(2048), rng.randbytes(rng.choice(LENGTHS)))
                    for _ in range(200)]
        wire = b"".join(b"AXDR" + ident.to_bytes(2, "little") + bytes([len(data)]) + data
                        for ident, data in expected)
        for _ in range(20):
            self.lib.Test_Reset()
            pos = 0
            while pos < len(wire):
                size = rng.randint(1, 64)
                self.feed(wire[pos:pos + size])
                pos += size
            self.assertEqual(self.received(), expected)

    def test_invalid_lengths_are_rejected_before_packing(self):
        output = ct.create_string_buffer(b"\xa5" * 71, 71)
        for length in range(256):
            with self.subTest(length=length):
                self.assertEqual(self.lib.AxDr_CANFD_Length_Valid(length), length in LENGTHS)
                if length not in LENGTHS:
                    msg = Message(0x1C1, length)
                    self.assertEqual(self.lib.Test_Pack(output, ct.byref(msg)), 0)
                    self.assertEqual(output.raw, b"\xa5" * 71)

    def test_invalid_length_header_does_not_block_next_frame(self):
        valid = b"AXDR\xc1\x01\x01\x2a"
        for length in range(256):
            if length in LENGTHS:
                continue
            wire = b"AXDR\xc1\x01" + bytes([length]) + valid
            for split in range(len(wire) + 1):
                self.lib.Test_Reset()
                self.feed(wire[:split])
                self.feed(wire[split:])
                self.assertEqual(self.received(), [(0x1C1, b"\x2a")], (length, split))

    def test_partial_header_and_payload_containing_header(self):
        wire = b"AXDR\xc1\x01\x04AXDR"
        self.feed(b"noiseAXDR\xff\xff\x00AXDR\x01\x00\x09" + wire[:3])
        self.assertEqual(self.received(), [])
        self.feed(wire[3:])
        self.assertEqual(self.received(), [(0x1C1, b"AXDR")])

    def test_write_response_bytes_and_status_mapping(self):
        response = Message.in_dll(self.lib, "Test_Response")
        for status in range(10):
            self.lib.Protocol_Write_Response(7, 0x0A12, status)
            self.assertEqual((response.id, response.length), (0x81, 6))
            self.assertEqual(bytes(response.data[:6]), bytes([7, 7, 2, status, 0x12, 0x0A]))
        for status, expected in enumerate((0, 3, 5, 4, 5, 6, 7)):
            self.lib.Protocol_Parameter_Write_Response(8, 0x0101, status)
            self.assertEqual((response.id, response.length), (0x81, 6))
            self.assertEqual(bytes(response.data[:6]), bytes([8, 7, 2, expected, 1, 1]))

    def test_disconnect_during_frame_dispatch(self):
        ct.c_uint32.in_dll(self.lib, "Test_Disconnect_On_Rx").value = 1
        self.feed(b"AXDR\x01\x00\x01\x2a")
        self.assertEqual(self.received(), [(1, b"\x2a")])

    def test_receive_thread_discards_partial_frame_after_disconnect_or_read_error(self):
        # 覆盖未连接、读取时断开、读取失败三条路径，旧半帧不能混入后续数据。
        for mode in range(3):
            with self.subTest(mode=mode):
                self.lib.Test_Reset()
                self.feed(b"AXDR\x01\x00\x40old")
                self.lib.Test_Run_Rx(mode)
                self.assertEqual(self.received(), [(1, b"\x2a")])

    def test_interrupt_notification_waits_for_thread_poll(self):
        isr = ct.c_uint32.in_dll(self.lib, "Test_Isr")
        mask = ct.c_uint32.in_dll(self.lib, "Test_Primask")
        wakes = ct.c_uint32.in_dll(self.lib, "Test_Wakes")
        self.lib.USB_Tx_Wake()
        self.assertEqual(wakes.value, 1)
        isr.value = 1
        for _ in range(3):
            self.lib.USB_Tx_Wake()
        self.assertEqual((wakes.value, mask.value), (1, 0))
        isr.value = 0
        mask.value = 1
        self.lib.USB_Tx_Poll()
        self.assertEqual((wakes.value, mask.value), (2, 1))
        self.lib.USB_Tx_Poll()
        self.assertEqual(wakes.value, 2)


if __name__ == "__main__":
    unittest.main()
