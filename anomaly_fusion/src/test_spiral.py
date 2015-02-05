from simulator.robot_simulator import RobotSpiralSimulator


simul = RobotSpiralSimulator(40, start_time=70, show_path=True, show_polyline=True, show_association=True, show_prediction=True)
simul.show()

# simul.save("anim.mp4")


