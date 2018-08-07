import os
from configparser import ConfigParser
from typing import Union, Any, Optional


def static_vars(**kwargs):
    """
    Decorator hack for introducing local static variables.
    :param kwargs: The declarations of the static variables like "foo=42".
    :return: The decorated function.
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


class ProjectConfig:
    """
    Contains settings specific to the project but not to the user.
    """
    configSection: str = 'BeamNG'
    widthOfScreenshot: int = 800
    numRequests: int = 10
    delayBeforeFirst: int = 3
    delay: int = 1


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
    return get_user_config.config.get(ProjectConfig.configSection, option)


if __name__ == '__main__':
    from beamngpy import BeamNGPy
    import time
    userpath: Union[Optional[object], Any] = get_user_config('userpath')
    binary: Union[Optional[object], Any] = get_user_config('x64_binary')
    with BeamNGPy('localhost', 64256, userpath=userpath, binary=binary, console=True) as bpy:
        bpy.load_scenario('levels/west_coast_usa/scenarios/basic_scenario.json')
        bpy.start_scenario()  # NOTE This method blocks until the restart button is pressed
        bpy.hide_hud()
        bpy.vcontrol({
            'throttle': 1.0,
            'steering': 0.5,
            'gear': 1
        })

        output_dir: Union[Optional[object], Any] = get_user_config('output_dir')
        time.sleep(ProjectConfig.delayBeforeFirst)
        for i in range(ProjectConfig.numRequests):
            vstate: object = bpy.get_vstate(ProjectConfig.widthOfScreenshot)
            with open(os.path.join(output_dir, f'{i:02}.png'), 'wb') as img:
                vstate['img'].save(img, format="PNG")
            time.sleep(ProjectConfig.delay)
