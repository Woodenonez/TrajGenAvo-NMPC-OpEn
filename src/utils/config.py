import yaml

""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""

required_config_params = {
    # Physical parameters and limitations of the motion model
    'vehicle_width': 'Vehicle width in meters',
    'vehicle_margin': 'Extra safe margin',
    'lin_vel_min': 'Vehicle contraint on the minimal velocity possible',
    'lin_vel_max': 'Vehicle contraint on the maximal velocity possible',
    'lin_acc_min': 'Vehicle contraint on the maximal linear retardatio',
    'lin_acc_max': 'Vehicle contraint on the maximal linear acceleration',
    'ang_vel_max': 'Vehicle contraint on the maximal angular velocity',
    'ang_acc_max': 'Vehicle contraint on the maximal angular acceleration (considered to be symmetric)',
    # Parameters specific to the solution algorithm
    'ts': 'Size of the time-step (sampling time)',
    'N_hor': 'The length of the receding horizon controller',
    'throttle_ratio': 'What "%" of the maximal velocity should be chosen',
    'num_steps_taken': 'How many steps should be taken from each mpc-solution. Range (1 - N_hor)',
    'vel_red_steps': 'Number of timesteps for braking when close to goal position',
    # Weights
    'lin_vel_penalty': 'Cost for linear velocity control action',
    'lin_acc_penalty': 'Cost for linear acceleration',
    'ang_vel_penalty': 'Cost angular velocity control action',
    'ang_acc_penalty': 'Cost angular acceleration',
    'qcte': 'Cost for cross-track-error from each line segment',
    'qp': 'Cost for position deviation (each time step vs reference point)',
    'qv': 'Cost for speed deviation each time step',
    'qtheta': 'Cost for each heading relative to the final refernce position',
    'qpN': 'Terminal cost; error relative to final reference position',   
    'qthetaN': 'Terminal cost; error relative to final reference heading',
    # Helper variables (Generally does not have to be changed)
    'ns': 'Number of states for the robot (x,y,theta)',
    'nu': 'Number of control inputs (linear and angular speeds)',
    'nq': 'Number of penlty parameters',
    'ndynobs': 'Number of variables per dynamic obstacle',
    'Ndynobs': 'Maximal number of dynamic obstacles',
    # Building options in the optimizer
    'build_type': 'Can have "debug" or "release"',
    'build_directory': 'Name of the directory where the build is created',
    'bad_exit_codes': 'Optimizer specific names',
    'optimizer_name': 'Optimizer type'
}


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class Configurator:
    def __init__(self, yaml_fp):
        self.prtname = '[CONFIG]'
        self.fp = yaml_fp
        print(f'{self.prtname} Loading configuration from "{self.fp}".')
        with open(self.fp, 'r') as stream:
            self.input = yaml.safe_load(stream)
        self.args = dotdict()

    def configurate_key(self, key):
        value = self.input.get(key)
        if value is None:
            print(f"{self.prtname} Can not find '{key}' in the YAML-file. Explanation is: '{required_config_params[key]}'.")
            raise RuntimeError(f'{self.prtname} Configuration is not properly set.')
        self.args[key] = value

    def configurate(self):
        for key in required_config_params:
            self.configurate_key(key)
        print(f'{self.prtname} Configuration done.')
        return self.args