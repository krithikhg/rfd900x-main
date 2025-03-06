import asyncio
from rfd900x import RFDConfig
from mavsdk import System
import serial

async def run():
    # Initialize the RFDConfig class
    rfd = RFDConfig()

    # List of supported baud rates
    supported_baud_rates = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 1000000]

    # Open the serial port (replace '/dev/cu.usbserial-AB0JNOD7' with your port and 115200 with your baud rate)
    port = 'COM3'
    baud = 115200

    if baud not in supported_baud_rates:
        print(f"Baud rate {baud} is not supported. Please use one of the supported baud rates: {supported_baud_rates}")
        return

    try:
        # Explicitly set serial port parameters
        rfd.port.port = port
        rfd.port.baudrate = baud
        rfd.port.bytesize = serial.EIGHTBITS
        rfd.port.parity = serial.PARITY_NONE
        rfd.port.stopbits = serial.STOPBITS_ONE
        rfd.port.timeout = 1
        rfd.port.xonxoff = False
        rfd.port.rtscts = False
        rfd.port.dsrdtr = False

        if rfd.open(port, baud):
            print("Connected to RFD 900x radio")

            # Enable MAVLink mode
            rfd.params['MAVLINK']['desVal'] = 1
            if rfd.writeOutParam('MAVLINK'):
                print("Successfully enabled MAVLink mode")

            # Create a MAVSDK System instance
            drone = System()
            await drone.connect(system_address=f"serial://{port}:{baud}")

            # Wait for the drone to connect
            async for state in drone.core.connection_state():
                if state.is_connected:
                    print(f"Drone discovered with UUID: {state.uuid}")
                    break

            # Example: Send a heartbeat message
            print("Sending heartbeat message")
            await drone.action.arm()
            print("Drone armed")

            # Close the serial port
            rfd.close()
            print("Closed the serial port")
        else:
            print("Failed to connect to RFD 900x radio")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(run())