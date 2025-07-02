import asyncio
from bleak import BleakScanner

async def monitor():
    while True:
        devices = await BleakScanner.discover(timeout=1.0)
        for d in devices:
            if d.name == "Wheelie":
                print(f"{d.address} Wheelie RSSI: {d.rssi} dBm")

if __name__ == "__main__":
    asyncio.run(monitor())
