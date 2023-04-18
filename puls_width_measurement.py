# Pulse width measurement example using PWMCounter.
#
# Measures pulse width of PWM generated on GP0
# 

from machine import Pin, PWM
from time import ticks_us, ticks_diff
from utime import sleep
from PWMCounter import PWMCounter

# Set PWM to output test signal
pwm = PWM(Pin(15))
# Set duty cycle to 25%
#pwm.duty_u16(1 << 14)
pwm.freq(50)





# 0 Grad
grad000 = 1638
# 90 Grad
grad090 = 4915
# 180 Grad
grad180 = 8192


# steigung = (RuderMax-Rudermin)/(ReglerMax-ReglerMin)
#(8192-1638)/(1999-979)=6554/1020= 6.42
# startwert = ReglerMin*steigung-RuderMin 
# 979*6.43 -1638


startwert = -4653
steigung = 6.43

#print('Position: Ganz Links (0 Grad)')
#pwm.duty_u16(grad000)
#sleep(2)

#print('Position: Mitte (90 Grad)')
#pwm.duty_u16(grad090)
#sleep(2)



# We'll use counter pin for triggering, so set it up.
in_pin = Pin(13, Pin.IN)
# Configure counter to count rising edges on GP15
counter = PWMCounter(13, PWMCounter.LEVEL_HIGH)
# Set divisor to 16 (helps avoid counter overflow)
counter.set_div(16)
# Start counter
counter.start()

last_state = 0
last_update = ticks_us()
while True:
    start = ticks_us()
    if ~(x := in_pin.value()) & last_state:
        # Print pulse width in us - should show 250 with default setup
        messwert = int (counter.read_and_reset() * 16 / 125 )
        print (messwert)
        zielwert = int (messwert*steigung+startwert)
        print (zielwert)
        pwm.duty_u16(zielwert)
       # sleep(1)
        # print((counter.read_and_reset() * 16) / 125)
    last_state = x
