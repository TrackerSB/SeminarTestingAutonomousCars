import os
import time
from configparser import ConfigParser

import beamngpy


class ProjectConfig:
    currentConfigSection = 'BeamNG'
    widthOfScreenshot = 800
    numScreenshots = 10


config = ConfigParser()
dirname = os.path.abspath(os.path.dirname(__file__))  # NOTE When inserting this line into the next, it may fail
config.read(os.path.join(dirname, '..', 'BeamNG.ini'))

userpath = config.get(ProjectConfig.currentConfigSection, 'userpath')
binary = config.get(ProjectConfig.currentConfigSection, 'x64_binary')
with beamngpy.beamng.BeamNGPy('localhost', 64256, userpath=userpath,
                              binary=binary, console=True) as bpy:
    bpy.load_scenario('levels/west_coast_usa/scenarios/basic_scenario.json')
    bpy.start_scenario()  # NOTE This method blocks until the restart button is pressed
    bpy.hide_hud()
    bpy.vcontrol({
        'throttle': 1.0,
        'steering': 0.5,
        'gear': 1
    })

    outputDir = config.get(ProjectConfig.currentConfigSection, 'output_dir')
    for i in range(ProjectConfig.numScreenshots):
        vstate = bpy.get_vstate(ProjectConfig.widthOfScreenshot)
        with open(os.path.join(outputDir, f'{i:02}.png'), 'wb') as img:
            vstate['img'].save(img, format="PNG")
        time.sleep(1)
