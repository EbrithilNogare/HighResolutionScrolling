import asyncio
from bleak import BleakScanner

async def monitor():
    rssi_values = []
    
    while True:
        devices = await BleakScanner.discover(timeout=1.0)
        for d in devices:
            if d.name == "Smooth scroller":
                rssi_values.append(d.rssi)
                avg_rssi = sum(rssi_values) / len(rssi_values)
                print(f"{d.address} Smooth scroller - Current: {d.rssi} dBm, Average: {avg_rssi:.1f} dBm ({len(rssi_values)} measurements)")

if __name__ == "__main__":
    asyncio.run(monitor())
