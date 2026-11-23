"""Read-only ELM327 session over BLE (Vgate iCar Pro "IOS-Vlink", service 0x18F0).

usage: python ble_elm.py ATZ ATE0 ATH0 ATSP6 0100 010B ...     (needs `pip install bleak`, e.g. in a venv)

The classic-BT radio ("Android-Vlink") drops the link on macOS; the BLE one works. Send only Mode 01
requests here (see docs/CAN-ANALYSIS.md); notify char 0x2AF0, write char 0x2AF1.
"""
import asyncio,sys
from bleak import BleakScanner,BleakClient
async def find():
    d=await BleakScanner.find_device_by_filter(lambda d,a:(d.name or a.local_name or '')=='IOS-Vlink',timeout=15)
    return d
class Elm:
    def __init__(s,c,w,n): s.c=c;s.w=w;s.buf=bytearray();s.ev=asyncio.Event()
    def on(s,_,data): s.buf+=data; s.ev.set()
    async def cmd(s,c,wait=6):
        s.buf.clear(); s.ev.clear()
        await s.c.write_gatt_char(s.w,(c+'\r').encode(),response=False)
        end=asyncio.get_event_loop().time()+wait
        while asyncio.get_event_loop().time()<end and not s.buf.endswith(b'>'):
            try: await asyncio.wait_for(s.ev.wait(),0.5)
            except asyncio.TimeoutError: pass
            s.ev.clear()
        return s.buf.decode(errors='replace').replace('\r','|').strip()
async def session(cmds):
    d=await find()
    if not d: print('IOS-Vlink not found'); return
    async with BleakClient(d,timeout=20) as c:
        w=n=None
        for sv in c.services:
            print('service',sv.uuid)
            for ch in sv.characteristics:
                print('  char',ch.uuid,ch.properties)
                if 'notify' in ch.properties: n=ch.uuid
                if 'write-without-response' in ch.properties or 'write' in ch.properties: w=w or ch.uuid
        n = '00002af0-0000-1000-8000-00805f9b34fb'
        w = '00002af1-0000-1000-8000-00805f9b34fb'
        e = Elm(c, w, n)
        await c.start_notify(n,e.on)
        for x in cmds: print(x,'->',await e.cmd(x))
if __name__=='__main__':
    asyncio.run(session(sys.argv[1:]))
