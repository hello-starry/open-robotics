import openrobotics as orb

if __name__ == "__main__":
    delta = orb.DeltaRobot()
    end_pos = delta.calc_forward_kinematics([10.0, 20.0, 30.0])
    joint_pos = delta.calc_inverse_kinematics(end_pos)
    print(end_pos)
    print(joint_pos)
