import asyncio
from vacuum_gripper import VacuumGripper

async def run():
    gripper = VacuumGripper('172.22.22.2')  # actual ip of the ur arm

    await gripper.connect()
    await gripper.activate()  

    await gripper.automatic_grip()
    obj_status = gripper.get_object_status()
    print(obj_status)

    # TODO: wait for a bit

    await gripper.automatic_release()
    obj_status = gripper.get_object_status()
    print(obj_status)

asyncio.run(run())
