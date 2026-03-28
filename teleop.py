import keyboard

vx = 0.0
vy = 0.0
vw = 0.0

speed = 0.5

def update():
    global vx, vy, vw

    vx = 0.0
    vy = 0.0
    vw = 0.0

    if keyboard.is_pressed('i'):
        vx = speed
    elif keyboard.is_pressed(','):
        vx = -speed

    elif keyboard.is_pressed('j'):
        vw = speed
    elif keyboard.is_pressed('l'):
        vw = -speed

    if keyboard.is_pressed('k'):
        vx = 0.0
        vy = 0.0
        vw = 0.0