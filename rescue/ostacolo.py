def ostacolo(robot):
    robot.motors.motors(50, 50)
    time.sleep(1)
    robot.motors.motors(-60, 60)
    time.sleep(2)

    while True:
        _, side_tof, _ = robot.get_tof_mesures()
        if side_tof < 100:
            robot.motors.motors(20, -60)
        else:
            robot.motors.motors(40, 60)