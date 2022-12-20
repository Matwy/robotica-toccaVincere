import sys
sys.path.insert(1, './motors-sensors')
import Robot
from linea import linea

def rescue():
    
    errore_linea, errore_angolo = linea()
    
    return

if __name__ == '__main__':
    rescue()