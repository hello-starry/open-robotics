import openrobotics as orb

if __name__ == "__main__":
    delta = orb.DeltaRobot()
    end_pos = delta.calc_forward_kinematics([10.0, 20.0, 30.0])
    joint_pos = delta.calc_inverse_kinematics(end_pos)
    print(end_pos)
    print(joint_pos)

    six_axis = orb.SixAxisRobot()
    joint_pos = six_axis.calc_forward_kinematics([0,0,0,0,0,0])
    print(f"The end_pos of six_axis is {joint_pos}")