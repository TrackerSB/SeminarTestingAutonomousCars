"""
Represents a basic application for communicating with BeamNG.research.
"""
import os
from configparser import ConfigParser
from multiprocessing import Process
from typing import Union, Any, Optional

from beamngpy import BeamNGPy


def static_vars(**kwargs):
    """
    Decorator hack for introducing local static variables.
    :param kwargs: The declarations of the static variables like "foo=42".
    :return: The decorated function.
    """

    def decorate(func):
        """
        Decorates the given function with local static variables based on kwargs.
        :param func: The function to decorate.
        :return: The decorated function.
        """
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func

    return decorate


project_config = {
    'configSection': 'BeamNG',
    'widthOfScreenshot': 800,
    'numRequests': 10,
    'delayBeforeFirst': 3,
    'delay': 1
}


def read_user_config() -> ConfigParser:
    """
    Returns the user specific configurations.
    :return: The user specific settings.
    """
    config: ConfigParser = ConfigParser()
    config.read(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'BeamNG.ini'))
    return config


@static_vars(config=read_user_config())
def get_user_config(option: str) -> object:
    """
    Returns the user specific setting.
    :param option: The option to retrieve.
    :return: The set value of the option.
    """
    return get_user_config.config.get(project_config['configSection'], option)


if __name__ == '__main__':
    import time

    userpath: Union[Optional[object], Any] = get_user_config('userpath')
    binary: Union[Optional[object], Any] = get_user_config('x64_binary')
    with BeamNGPy('localhost', 64256, userpath=userpath, binary=binary, console=True, annotationconfig=True) as bpy:
        # attach_cleanup_process(bpy)
        bpy.load_scenario('levels/west_coast_usa/scenarios/basic_scenario.json')
        bpy.start_scenario()  # NOTE This method blocks until the restart button is pressed
        # bpy.hide_hud()
        #bpy.vcontrol({
        #    'throttle': 1.0,
        #    'steering': 0.5,
        #    'gear': 1
        #})

        output_dir: Union[Optional[object], Any] = get_user_config('output_dir')
        time.sleep(project_config['delayBeforeFirst'])
        for i in range(project_config['numRequests']):
            vstate: dict = bpy.get_vstate(project_config['widthOfScreenshot'])
            with open(os.path.join(output_dir, f'{i:02}.png'), 'wb') as img:
                # print(vstate.keys())
                vstate['img'].save(img, format="PNG")
            time.sleep(project_config['delay'])

        print("Finished")
        input("Press enter to exit.")
