from enum import Enum
import asyncio

from rtde_control import RTDEControlInterface as RTDEControl
from serl_spacemouse import SpaceMouseExpert

from gripper import Gripper
from vacuum_gripper import VacuumGripper

import numpy as np

class GripperType(Enum):
    """ """

    TWO_FINGER = 0
    VACUUM = 1

async def control_2f_gripper(gripper, buttons):
    if buttons[0]:
        await gripper.move_and_wait_for_pos(255, 255, 255)

    if buttons[1]:
        await gripper.move_and_wait_for_pos(0, 255, 255)

async def control_vacuum_gripper(gripper, buttons):
    if buttons[0]:
        print('trying to grip')
        #await gripper.automatic_grip()
        await gripper.advanced_grip(30, 78, 50)

    if buttons[1]:
        print('trying to release')
        await gripper.automatic_release()

async def test_spacemouse_ur5e_forcemode(gripper_type = GripperType.TWO_FINGER):
    rtde_c = RTDEControl("172.22.22.2")
    if gripper_type == GripperType.TWO_FINGER:
        gripper = Gripper('172.22.22.2')  # ip of the arm (which handles the communication with the gripper)
    else:
        gripper = VacuumGripper('172.22.22.2')  # ip of the arm (which handles the communication with the gripper)

    task_frame = [0, 0, 0, 0, 0, 0]
    selection_vector = [1, 1, 1, 1, 1, 1]
    force_type = 2
    limits = [2, 2, 1.5, 1, 1, 1]

    # Execute 500Hz control loop each cycle is 2ms
    sm = SpaceMouseExpert()

    await gripper.connect()
    await gripper.activate()  # calibrates the gripper

    while True:
        action, buttons = sm.get_action()

        t_start = rtde_c.initPeriod()
        print(action)
        action[:3] = action[:3] * 50.
        action[3:] = action[3:] * 10.

        rtde_c.forceMode(task_frame, selection_vector, action, force_type, limits)
        if gripper_type == GripperType.TWO_FINGER:
            await control_2f_gripper(gripper, buttons)
        else:
            await control_vacuum_gripper(gripper, buttons)

        rtde_c.waitPeriod(t_start)

    rtde_c.forceModeStop()
    rtde_c.stopScript()

async def test_spacemouse_ur5e_jog(gripper_type = GripperType.TWO_FINGER):
    rtde_c = RTDEControl("172.22.22.2")
    if gripper_type == GripperType.TWO_FINGER:
        gripper = Gripper('172.22.22.2')  # ip of the arm (which handles the communication with the gripper)
    else:
        gripper = VacuumGripper('172.22.22.2')  # ip of the arm (which handles the communication with the gripper)

    sm = SpaceMouseExpert()

    await gripper.connect()

    # this part is not necessrily needed for the vacuum gripper, as we do not need to calibrate it
    await gripper.activate()  # calibrates the gripper

    N = 100
    forces = np.zeros((N, 6))
    is_in_contact = np.zeros((N, 1))
    idx = 0

    while True:
        action, buttons = sm.get_action()
        t_start = rtde_c.initPeriod()

        action[3:] = action[3:] * 0.05
        action[:3] = action[:3] * 0.5

        rtde_c.jogStart(action)
        if gripper_type == GripperType.TWO_FINGER:
            await control_2f_gripper(gripper, buttons)
        else:
            await control_vacuum_gripper(gripper, buttons)

        # log new data
        idx = (idx + 1) % forces.shape[0]
        forces[idx, :] = rtde_c.getJointTorques()

        # argument filters the contact detection by direction, all 0 lead to 'detect every contact'
        is_in_contact[idx, :] = rtde_c.toolContact([0, 0, 0])

        print(forces[idx, :])
        #print(is_in_contact[idx, :])

        rtde_c.waitPeriod(t_start)
        
    rtde_c.jogStop()
    rtde_c.stopScript()

def main():
    #asyncio.run(test_spacemouse_ur5e_forcemode(GripperType.VACUUM))
    asyncio.run(test_spacemouse_ur5e_jog(GripperType.VACUUM))

if __name__ == "__main__":
    main()
