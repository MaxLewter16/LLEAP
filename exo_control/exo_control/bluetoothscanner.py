import asyncio
from bleak import BleakScanner

class BLEScannerNode:
    def __init__(self):
        pass

    async def run(self):
        scanner = BleakScanner()
        while True:
            devices = await scanner.discover()
            for device in devices:
                print(f"Device: {device.name}, Address: {device.address}, RSSI: {device.rssi}")
            await asyncio.sleep(5)

def main():
    node = BLEScannerNode()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.run())

if __name__ == "__main__":
    main()