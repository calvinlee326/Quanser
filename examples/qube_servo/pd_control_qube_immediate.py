## example_pd_control_qube_hardware_task.py
# This example sets up a PD controller to control the position of the QUBE Servo Disk.
# This example uses either a virtual or Physical Qube Servo 2 or Physical Qube Servo 3 device,
# in an immediate IO mode where you have to handle timing yourself explicitly.
# Use a time-based sleep to help with timing, but note that this mode is not recommended.
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

# imports
from threading import Thread
import signal
import time
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import SignalGenerator
from pal.utilities.scope import Scope

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

#region: Setup
simulationTime = 10 # will run for 10 seconds
sampleTime = 0.002 # 500 Hz
color = np.array([0, 1, 0], dtype=np.float64)

squareWave = SignalGenerator().square(np.pi/4, 5)
next(squareWave)

scopePosition = Scope(
    title='Measured vs Desired Position',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopePosition.attachSignal(name='Measured Position',  width=1)
scopePosition.attachSignal(name='Desired Position', color=[76,114,176], width=2, lineStyle='--')

scopeVoltage = Scope(
    title='Commanded Voltage',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Voltage')
scopeVoltage.attachSignal(name='Voltage')
#endregion

# Code to control the Qube Hardware
# CHANGE qubeVersion, hardware and pendulum VARIABLES FOR DIFFERENT SETUPS
def control_loop():

    # set as 2 or 3 if using a Qube Servo 2 or 3 respectively
    qubeVersion = 3

    # Set as 0 if using virtual Qube Servo 2
    # Set as 1 if using physical Qube Servo 2 or 3
    hardware = 1

    # Set as 0 if using virtual QUBE 2 - DC Motor
    # Set as 1 if using virtual QUBE 2 - Pendulum
    # value is not used for Qube 3.
    pendulum = 0

    k_p, k_d = (4.000, 0.175)

    if qubeVersion == 2:
        QubeClass = QubeServo2
    else:
        QubeClass = QubeServo3

    with QubeClass(hardware = hardware, pendulum = pendulum, readMode=0) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        while timeStamp < simulationTime and not KILL_THREAD:
            start = elapsed_time()

            # Read sensor information
            myQube.read_outputs()

            # Command square wave
            desiredPosition = squareWave.send(timeStamp)

            # Controller
            motorPosition = myQube.motorPosition
            voltage = k_p * (desiredPosition - motorPosition) - k_d * ( myQube.motorSpeed )

            # Write commands
            myQube.write_led(color)
            myQube.write_voltage(voltage)

            scopePosition.sample(timeStamp, [motorPosition, desiredPosition])
            scopeVoltage.sample(timeStamp, [voltage])

            end = elapsed_time()

            computationTime = (end - start) % sampleTime

            time.sleep(sampleTime - computationTime)
            timeStamp = elapsed_time()


# Setup data generation thread and run until complete
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):

    # This must be called regularly or the scope windows will freeze
    # Must be called in the main thread.
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
