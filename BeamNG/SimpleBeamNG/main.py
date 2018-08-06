import os
import time
from configparser import ConfigParser

import beamngpy

currentConfigSection = 'BeamNG'

config = ConfigParser()
dirname = os.path.abspath(os.path.dirname(__file__))  # NOTE When inserting this line into the next, it may fail
config.read(os.path.join(dirname, '..', 'BeamNG.ini'))

# FIXME First execution does not work
userpath = config.get(currentConfigSection, 'userpath')
binary = config.get(currentConfigSection, 'x64_binary')
with beamngpy.beamng.BeamNGPy('localhost', 64256, userpath=userpath,
                              binary=binary, console=True) as bpy:
    bpy.load_scenario('levels/west_coast_usa/scenarios/basic_scenario.json')
    time.sleep(10)
    bpy.start_scenario()  # FIXME Why is this called to early?
    bpy.hide_hud()
    bpy.vcontrol({
        'throttle': 1.0,
        'steering': 0.5,
        'gear': 1
    })

    outputDir = config.get(currentConfigSection, 'output_dir')
    for i in range(10):
        vstate = bpy.get_vstate(800)
        with open(os.path.join(outputDir, f'{i:02}.png'), 'wb') as img:
            vstate['img'].save(img, format="PNG")
        time.sleep(1)
