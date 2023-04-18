# Steuerung von 3 Schottelantrieben - Rotortug
#
# 2 Bugantriebe Synchron angesteuert
# 1 Heckantrieb
#
# Auslesen der Empfängersignale für Kreuzknüppel links = ReglerEinBug, RuderEinBug
# Auslesen der Empfängersignale für Kreuzknüppel rechts = ReglerEinHeck, RuderEinHeck
#
# Im Detail:
#
# ReglerEinBug
# ReglerEinHeck
# RuderEinBug
# RuderEinHeck
#
# Absolutwerte  (ermitteln z.B. mit diesem Programm lauffähig am Computer - todo: am See
# Min und Max werte für alle 4 Werte des Kreuzknüppels
#
# Normieren der ReglerEingangssignale auf -100...100%
# Normieren der RuderEingangssignale auf 0...180 Grad (0=Links, 90=Nord, 180 = Rechts)
# Vektorsteuerung tbd
#
#Getriebeübersetzung 1:2 180 Grad Servo == 360 Grad Schottel
#
# Berechnen der Signale für ReglerAus (ReglerAusX = ABS(ReglerEinX)
# Berechnen der Signale für RuderAus (RuderAusX = RuderEinX (ReglerEinX>0); RuderAusX = 360 - RuderEinX (ReglerEinX<0)      
# Deadband um Zittern der Servos zu verhindern:     5 < ReglerEinX < -5      RuderAusX: no change    ReglerAusX = 0
#
# Ausgabe berechnete Werte
 
# Quellen:
# https://github.com/phoreglad/pico-MP-modules/tree/main/PWMCounter
# Pico hardware allows only odd numbered GPIOs to be used in counter mode.
# Some pins share the same PWM slice (see list below) and if they are
# used in counting mode at the same time the signal seen by counter is a logical OR of both inputs.
 

from machine import Pin, PWM
from time import ticks_us, ticks_diff
from utime import sleep
from PWMCounter import PWMCounter

# Set PWM to output test signal

ReglerAusBug1 = PWM(Pin(0))
# ReglerAusBug2 parallel zu ReglerAusBug1 geschaltet -tbd platinenseitig
RuderAusBug1 = PWM(Pin(4))
# RuderAusBug2 parallel zu RuderAusBug1 geschaltet -tbd platinenseitig
ReglerAusHeck = PWM(Pin(2))
RuderAusHeck = PWM(Pin(6))

ReglerAusBug1.freq(50)
ReglerAusHeck.freq(50)
RuderAusBug1.freq(50)
RuderAusHeck.freq(50)


#Messwerte aus DX9/OrangeR615X

throttlemin = 979
throttlemax = 1999
throttlemitte = 1486


rudermin = 979
rudermax = 1999
rudermitte = 1486



# deadband hier soll der regle auf null gehen und die hysterese für einen 2-punktregler für das umstueren der schottel liefern
deadbandmax =  throttlemitte +25
deadbandmin = throttlemitte -25
# wohin zeigt der schottel - für hysterese 0 == vorwaerts 1 == rueckwaerts
schottelstatus = 0
# schottelstatus 2 für Berechnung des ziewertes der servos für umkehrfahrt 
schottelstatus2 = -1


# 180 grad servos
# 0 Grad
grad000 = 1638

grad045 = 3276  # 1638*2

# 90 Grad
grad090 = 4915  #1638*3 =4914

grad135 = 6552

# 180 Grad
grad180 = 8192


# "90" grad regler
# +100%
reglermin = 3016
# Nullstellung
reglermitte = 4658
# -100%
reglermax  = 6291



steigung = (grad090-grad000)/(rudermax-rudermin)
startwert = grad000 - steigung * rudermin



reglersteigung =  (reglermax-reglermin)/(throttlemax-throttlemin)
reglerstartwert = reglermin - reglersteigung*throttlemin




# We'll use counter pin for triggering, so set it up.
#in_pin = Pin(13, Pin.IN)
ReglerEinBug = Pin(13, Pin.IN)
RuderEinBug = Pin(15, Pin.IN)
ReglerEinHeck = Pin(9, Pin.IN)
RuderEinHeck = Pin(11, Pin.IN)

# Configure counter to count rising edges on GPioX

ReglerEinBugCounter = PWMCounter(13, PWMCounter.LEVEL_HIGH)
RuderEinBugCounter = PWMCounter(15, PWMCounter.LEVEL_HIGH)
ReglerEinHeckCounter = PWMCounter(11, PWMCounter.LEVEL_HIGH)
RuderEinHeckCounter = PWMCounter(9, PWMCounter.LEVEL_HIGH)
# Set divisor to 16 (helps avoid counter overflow)
ReglerEinBugCounter.set_div(16)
RuderEinBugCounter.set_div(16)
ReglerEinHeckCounter.set_div(16)
RuderEinHeckCounter.set_div(16)
# Start counter
ReglerEinBugCounter.start()
RuderEinBugCounter.start()
ReglerEinHeckCounter.start()
RuderEinHeckCounter.start()

ReglerEinBugLast_state = 0
RuderEinBugLast_state = 0
ReglerEinHeckLast_state = 0
RuderEinHeckLast_state = 0
#last_update = ticks_us()


# globale messwert variablen
ReglerEinBugMesswert =0
ReglerEinBugZielwert =0


while True:
    #start = ticks_us()
    if ~(x := ReglerEinBug.value()) & ReglerEinBugLast_state:
        # Print pulse width in us - should show 250 with default setup
        ReglerEinBugMesswert = int (ReglerEinBugCounter.read_and_reset() * 16 / 125 )
        # Abfrage ob der regler unter 0% ist für invertierung des Reglersignals: er läuft wieder vorwaärts
        
        if ReglerEinBugMesswert < throttlemitte:
            schottelstatus = 1
            schottelstatus2 = 1
            ReglerEinBugMesswert = throttlemitte + (throttlemitte-ReglerEinBugMesswert)
        else:
            schottelstatus = 0
            schottelstatus2 = -1
          
        #print ("ReglerEinBugMesswert " , ReglerEinBugMesswert)
        #print (ReglerEinBugMesswert)
        zielwert = int (ReglerEinBugMesswert*reglersteigung+reglerstartwert)
        #print ("Zielwert " , zielwert)
        ReglerAusBug1.duty_u16(zielwert)
    ReglerEinBugLast_state = x
    
    if ~(x := RuderEinBug.value()) & RuderEinBugLast_state:
        # Print pulse width in us - should show 250 with default setup
        RuderEinBugMesswert = int (RuderEinBugCounter.read_and_reset() * 16 / 125 )
        print ("RuderEinBugMesswert" , RuderEinBugMesswert)
        #print (RuderEinBugMesswert)
        
        zielwert = ((grad000+grad180) * schottelstatus) - schottelstatus2 * int (RuderEinBugMesswert*steigung+startwert)
        #print ("Status " , schottelstatus , "schottelstatus2" , schottelstatus2 )
        #print ("Startwert " , startwert , "Steigung" , steigung )
        print ("Zielwert Servo" , zielwert)
        RuderAusBug1.duty_u16(zielwert)   
    RuderEinBugLast_state = x
    
    if ~(x := ReglerEinHeck.value()) & ReglerEinHeckLast_state:
        # Print pulse width in us - should show 250 with default setup
        ReglerEinHeckMesswert = int (ReglerEinHeckCounter.read_and_reset() * 16 / 125 )
        #print ("ReglerHeckMesswert " , ReglerEinHeckMesswert)
        #print (ReglerEinHeckMesswert)
        zielwert = int (ReglerEinHeckMesswert*reglersteigung+reglerstartwert)
        #print (zielwert)
        ReglerAusHeck.duty_u16(zielwert)
    ReglerEinHeckLast_state = x
    
    if ~(x := RuderEinHeck.value()) & RuderEinHeckLast_state:
        # Print pulse width in us - should show 250 with default setup
        RuderEinHeckMesswert = int (RuderEinHeckCounter.read_and_reset() * 16 / 125 )
        #print ("RuderEinHeckMesswert" , RuderEinHeckMesswert)
        #print (RuderEinHeckMesswert)
        zielwert = int (RuderEinHeckMesswert*steigung+startwert)
        #print (zielwert)
        RuderAusHeck.duty_u16(zielwert)   
    RuderEinHeckLast_state = x
        



