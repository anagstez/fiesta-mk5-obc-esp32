"""Open the board's USB-Serial/JTAG port and wait until the firmware is up.

Opening the port can reset the chip (USB_UART_CHIP_RESET): lines sent right after open() are lost
while it boots. So keep DTR/RTS low, then wait for the 'obc: ready' log line before sending commands.
The port name changes between sockets/boards (usbmodem101, usbmodem1101, usbmodem<serial>): default is the first match.
"""
import glob, os, time
import serial


def open_board(port=None, timeout=1):
    port = port or os.environ.get("OBC_PORT") or (sorted(glob.glob("/dev/cu.usbmodem*")) or ["/dev/cu.usbmodem101"])[0]
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, timeout
    s.dtr = s.rts = False
    s.open()
    t = time.time()
    while time.time() - t < 6:                  # no line within 6 s: the firmware was already running
        if b"obc: ready" in s.readline():
            time.sleep(1.0)
            break
    s.reset_input_buffer()
    return s
