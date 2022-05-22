import yaml

""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""

class Configurator:

    class dotdict(dict):
        """dot.notation access to dictionary attributes"""
        __getattr__ = dict.get
        __setattr__ = dict.__setitem__
        __delattr__ = dict.__delitem__

    def __init__(self, yaml_fp):
        self.prtname = '[CONFIG]'
        self.fp = yaml_fp
        print(f'{self.prtname} Loading configuration from "{self.fp}".')
        with open(self.fp, 'r') as stream:
            self.input = yaml.safe_load(stream)

    def configurate(self):
        self.args = self.dotdict()
        for key, value in self.input.items():
            self.args[key] = value
        print(f'{self.prtname} Configuration done.')
        return self.args

