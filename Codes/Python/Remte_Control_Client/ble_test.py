import asyncio
import logging
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

logger = logging.getLogger(__name__)

def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Simple notification handler which prints the data received."""
    logger.info("%s: %r", characteristic.description, data)

async def main():
    target_device_name = 'Arduino LED'
    devices = await BleakScanner.discover()
    target_found = False
    for d in devices:
        print(d)
        if d.name == target_device_name:
            print("Matching device found")
            target_found = True
            break
    target_device = d

    if target_found:
        async with BleakClient(target_device) as client:
            for service in client.services:
                logger.info("[Service] %s", service)
                for char in service.characteristics:
                    if "read" in char.properties:
                        try:
                            value = await client.read_gatt_char(char.uuid)
                            extra = f", Value: {value}"
                        except Exception as e:
                            extra = f", Error: {e}"
                    else:
                        extra = ""

                    if "write-without-response" in char.properties:
                        extra += f", Max write w/o rsp size: {char.max_write_without_response_size}"

                    logger.info(
                        "  [Characteristic] %s (%s)%s",
                        char,
                        ",".join(char.properties),
                        extra,
                    )

                    for descriptor in char.descriptors:
                        try:
                            value = await client.read_gatt_descriptor(descriptor.handle)
                            logger.info("    [Descriptor] %s, Value: %r", descriptor, value)
                        except Exception as e:
                            logger.error("    [Descriptor] %s, Error: %s", descriptor, e)

            await client.start_notify("7b0db1df-67ed-46ef-b091-b4472119ef6d", notification_handler)
            await asyncio.sleep(1.0)
            await client.write_gatt_char('99924646-b9d6-4a51-bda9-ef084d793abf', "Testing 111\0".encode())
            await client.write_gatt_char('99924646-b9d6-4a51-bda9-ef084d793abf', "Testing 22\0".encode())
            await client.write_gatt_char('99924646-b9d6-4a51-bda9-ef084d793abf', "Testing 3\0".encode())


log_level = logging.INFO
logging.basicConfig(
    level=log_level,
    format="%(levelname)s: %(message)s",
)
asyncio.run(main())
print("done")