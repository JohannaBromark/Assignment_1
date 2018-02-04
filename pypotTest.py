from pypot.vrep import VrepIO

vrepIO = VrepIO()

vrepIO.add_sphere("boll", [1,1,0.25], [1, 1, 1], 1)


vrepIO.start_simulation()


vrepIO.stop_simulation()

vrepIO.close()
#vrepIO.add_cube("Kub", [1,1,0.25], [1,1,1], 1)
