from ctypes.wintypes import SIZE
import serial
import musicalbeeps
from playsound import playsound
from PIL import Image

def play_notes(note):
    player = musicalbeeps.Player(volume = 1, mute_output = False)

    note_list = ['A', 'B', 'C', 'D', 'E', 'F', 'G']

    if note in note_list:
        player.play_note(note, 0.25)

    elif note == 'W':
        im.show() 
        playsound(vic_sound)

im = Image.open(r'serial_clue.png') 
vic_sound = 'victory.wav'

mc_baudrate = 115200
name = '/dev/tty.usbmodem14103'

ser = serial.Serial(port=name, baudrate=mc_baudrate)

if ser.is_open:
    while True:
        size = ser.inWaiting()

        if size:
            data = ser.read(size)
            data = data.decode("utf-8")
            win_val = play_notes(data)
                
else:
    print('ser not open')

