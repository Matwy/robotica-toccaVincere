# camera and sensorsss
from PiVideoStream import PiVideoStream

#inizializzo la classe che prende i frame dalla cam in un thread separato
cam_stream = PiVideoStream().start()

output = None
LARGHEZZA = 320
ALTEZZA = 240