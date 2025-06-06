import asyncio
from control import ControllerNode,sensors,control,logger # Adjust import path if needed

async def main():
    # Initialize controller hardware and sensors
    controller = ControllerNode()
    await controller._async_init()

    # Start your control and sensor loops concurrently
    await asyncio.gather(
        control(controller),
        sensors(controller)
        serial_messages(controller)
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Shutting down.") 
