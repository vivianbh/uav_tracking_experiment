from pynput import keyboard

class Eo_rate:
    PanRight='d'
    PanLeft='a'
    PanStop='q'
    TiltUp='w'
    TiltDown='s'
    TiltStop='e'

usercmd = ''

def on_press(key):
    global usercmd
    usercmd=''
    try:
        #print('alphanumeric key {0} pressed'.format(key.char))
        usercmd = key.char
    except AttributeError:
        None
        #print('special key {0} pressed'.format(key))

def on_release(key):
    #print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        return False

#for testing
'''
while True:
    with keyboard.Listener(on_press = on_press, on_release = on_release) as listener:
        listener.join()
'''