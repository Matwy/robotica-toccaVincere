import sys
sys.path.insert(1, './motors-sensors')
import Robot
from linea import linea

def rescue(robot):
    
    frame = robot.get_frame()
    errore_linea, errore_angolo = linea(frame)
    print(errore_linea, errore_angolo)
    return

if __name__ == '__main__':
    robot = Robot()
    rescue(robot)