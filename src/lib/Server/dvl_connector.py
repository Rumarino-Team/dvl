from random import Random
from dvl_data import DVL_DATA
from datetime import datetime
from math import nan, isnan
from dvl.system import OutputData
from dvl.dvl import Dvl
from displacement_integration import displacement
from copy import deepcopy


DEFAULT_MAX_SERIAL_PORTS = 10
DEFAULT_LINUX_SERIAL_PORT = '/dev/ttyUSB' # Default port for jetson, usually USB0
DEFAULT_JETSON_SERIAL_PORT = '/dev/ttyUSB0'
# DEFAULT_LINUX_SERIAL_PORT = '/dev/tty' # Default port usually in linux computer
DEFAULT_DVL_PORT = 115200 # Currently not being used

def _check_for_nan(data: DVL_DATA, debug=False): # I don't think this is needed, if there is nan on json it would be 'invalid' but it produces no error receiving on the node.    
    '''NaN values give errors in json format, therefore are converted to -1.0'''
    if debug:
        for attr, val in vars(data).items():
            if isinstance(val, float) and isnan(val):
                setattr(data, attr, -1.0)
    return vars(data)

def _get_time_s_dvldata(dvl_data: dict):
    """Returns time in seconds from the given DVL_Data var dictionary.
    
    Args:
        dvl_data: dict -- dictionary containing all variables from a DVL_Data instance

    Return:
        float: value representing the time from a dvl_data in seconds
    """
    curr_date = datetime(dvl_data['year'], dvl_data['month'],
                         dvl_data['day'], dvl_data['hour'],
                         dvl_data['minute'], dvl_data['second'])
    return curr_date.timestamp()
    

class DVLHelper:
    """Class used to determine the displacement in position from dvl messages being transmitted.
    
    The idea of this class is to verify the data from DVL_Data messages, verifying that
    provided velocities are valid values. If the values from the data instances are
    valid, a copy of the instance is stored to then wait for another valid data instance
    to then be able to compare both of the instances and obtain the change in position
    according to velocity and time.
    It will always compare the latest two valid data message instances, it does not matter
    how new or old they are. For example if the class instance has already one data instance
    stored it will wait until the next data instance with VALID data appears no matter how
    long it took to appear and then it will compare these two instances to obtain the change
    in position.
    Example:
    1st valid data instance stored.
    2nd next valid data instance appeared after 100000 hours of runtime
    These two data instances will be compared.
    
    """

    VELOCITY_VAR_NAMES = ('vel_x', 'vel_y', 'vel_z')

    def __init__(self):
        self.past_dvl_data = []        

    def _check_for_valid_data(self, dvl_data: dict) -> bool:
        """Checks for valid data in message dictionary.
        No None, nan or instances other than floats are allowed.
        Variables checked are: "vel_x", "vel_y", "vel_z" from the dictionary.

        Args:
            dvl_data: dict -- vars dictionary from a DVL_Data instance
        
        Return:
            True:   if data is valid and a copy of the data is added to the class internal storage
            False:  if data is not valid
        """
        if dvl_data:
            for var in DVLHelper.VELOCITY_VAR_NAMES:
                if dvl_data[var] is None or isnan(dvl_data[var] or not isinstance(dvl_data[var], float)):
                    return False
            self.past_dvl_data.append(deepcopy(dvl_data))
            return True
        else:
            return False

    def calculate_displacement(self):
        """Calculates the displacement of position. Function first checks if it
        has enough data to obtain displacement. If it does not have enough data
        all values will be 0, otherwise the corresponding values will be assigned.

        Dictionary will always contain the following keys:
        x
        y
        z        

        Return:
            dict: dictionary containing the values of position displacement.
        """
        if len(self.past_dvl_data) != 0 and not len(self.past_dvl_data) % 2:
            f_data = self.past_dvl_data.pop()
            i_data = self.past_dvl_data.pop()
            displacement_res = displacement((i_data['vel_x'], i_data['vel_y'], i_data['vel_z']),
                                            (f_data['vel_x'], f_data['vel_y'], f_data['vel_z']),
                                            (_get_time_s_dvldata(i_data), _get_time_s_dvldata(f_data)))            
            return dict(zip(('x', 'y', 'z'), displacement_res))
        else:
            return {'x':0, 'y':0, 'z':0}


class DVLDevice:
    """ Parent class used to create different DVL devices.
    This class should not be used directly.
    """

    def connect(self):
        """Connects to the DVL system
        Should be overriden in a child class.
        """
        raise NotImplementedError

    def get_data(self):
        """Gets the data from the dvl device. This method should not be modified."""
        return {'DVL_Data' : _check_for_nan(self.get_dvl_data())}

    def get_dvl_data(self):
        """Used to retrieve the data from the DVL device according to the device being used.
        This method should be overriden in a child class.
        """
        raise NotImplementedError


class WayfinderDVL(DVLDevice):
    
    def __init__(self, serial_port_path: str = DEFAULT_LINUX_SERIAL_PORT):
        # Find a way to tell the dvl to stop/start and exit/close/release resources
        self.dvl: Dvl = Dvl()
        self.dvl_data = DVL_DATA()
        self.dvl_helper = DVLHelper()
        self.current_port = serial_port_path
        self._register_callback_function()


    def connect(self, device_port_path=DEFAULT_JETSON_SERIAL_PORT):
        #TODO Has not been tested
        """Connects to the DVL device.

        Args:
            device_port_path: str -- full path to the device USB port, Linux is usually "/dev/ttyUSB#", # being the port number being used, e.g. "/dev/ttyUSB0" or "/dev/ttyUSB0"


        Return:
            True : if succesful connection
            False: if connection failed
        """
        if self.dvl.connect(device_port_path):
            print(f'Connected to port: {device_port_path}\nGetting device setup...')
            if not self.dvl.get_setup():
                print('Failed to get system setup.\nSystem will disconnect.')
                self.disconnect()
            else:
                # Print system setup
                print(self.dvl.system_setup)
                # Modify system setup structure to set software trigger
                dvl_setup = self.dvl.system_setup
                dvl_setup.software_trigger = 1
                if not self.dvl.set_setup(dvl_setup):
                    print('Failed to set system setup\nDisconnecting...')
                    self.disconnect()
                # Exit command mode, to start pinging
                if not self.dvl.exit_command_mode():
                    print('Failed to exit command mode.\nDisconnecting...')
                    self.disconnect()
                else:
                    print('DVL device susscesfully connected.')
                    return True
            return False


    '''
    def connect(self):
        """Connects to the DVL device.
        
        Return:
            True : if successful connection
            False: if connection failed.
        """
        # TODO Overcomplicated connect is not necessary, Refactor or create simpler connect method.
        # Method currently tries to connect to the current port set and tries it with 10 different numbers.
        # E.g. /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2, /dev/ttyUSB3, /dev/ttyUSB4...
        for port_num in range(DEFAULT_MAX_SERIAL_PORTS):
            if self.dvl.connect(self.current_port + str(port_num)):
                self.current_port += str(port_num)
                print(f'Connected to port: {self.current_port}')
                if not self.dvl.get_setup():
                    print('Failed to get system setup.\nSystem will disconnect.')
                    self.disconnect()
                else:
                    # Print system setup
                    print(self.dvl.system_setup)
                    # Modify system setup structure to set software trigger
                    dvl_setup = self.dvl.system_setup
                    dvl_setup.software_trigger = 1
                    if not self.dvl.set_setup(dvl_setup):
                        print('Failed to set system setup.')
                        self.disconnect()
                    # Exit command mode, to start pinging
                    if not self.dvl.exit_command_mode():
                        print('Failed to exit command mode.')
                        self.disconnect()
                    else:
                        print('Dvl device fully connected.')
                        return True
                break

        if not self.dvl.is_connected():
            print(f'Tried to connect with {DEFAULT_MAX_SERIAL_PORTS} ports and failed.')

        return False
    '''

    def disconnect(self):
        """Disconnects the DVL device."""
        self.dvl.disconnect()
        if self.dvl.is_connected:
            print('Dvl device could not disconnect.')

    def start_logging(self):
        # Collect data - make sure working folder exists
        # print("Data logged to {0}".format(DVL.get_log_file_name()))
        # print("Failed to open {0} - make sure it is not used by any other program".format(PORT))
        raise NotImplementedError

    def _register_callback_function(self):
        self.dvl.register_ondata_callback(self._update_dvl_data)

    def _unregister_callback_functions(self):
        self.dvl.unregister_all_callbacks()

    def _update_dvl_data(self, output_data: OutputData, obj):
        """Function being called each time the DVL device is pinged.
        The DVL_Data instance is updated with the values received from the DVL.
        """
        del obj # Used in examples of code provided, no idea why
        if output_data is not None:
            self.dvl_data.clear_data()
            self.dvl_data.prepare_data(vars(output_data))
            self.dvl_helper._check_for_valid_data(vars(self.dvl_data))
            self.dvl_data.displacement = self.dvl_helper.calculate_displacement()
        else:
            print('Received no data.')

    def get_dvl_data(self):
        """Pings the DVL device and returns the DVL_Data dictionary."""
        if not self.dvl.send_software_trigger():
            print('Failed to send data request to Dvl')
        return vars(self.dvl_data)




class DVLDummyDevice(DVLDevice):

    def __init__(self) -> None:
        self.rand = Random()        
        self.dvl_data = DVL_DATA()
        self.dvl_helper = DVLHelper()

    def connect(self):
        """Dummy connect will always return True when called."""
        return True        

    def get_dummy_data(self):
        """Returns the internal DVL_Data instance."""
        self.prepare_random_data()
        return self.dvl_data

    def get_dummy_data_as_json(self): # Not being used currently.
        """Returns the DVL_Data in json format."""
        self.prepare_random_data()
        return {'DVL_Data' : vars(self.dvl_data)} 

    def get_dvl_data(self):
        """Prepares a random DVL_Data and returns it."""
        self.prepare_random_data()
        self.dvl_helper._check_for_valid_data(vars(self.dvl_data))
        self.dvl_data.displacement = self.dvl_helper.calculate_displacement()
        return self.dvl_data
    
    def prepare_random_data(self):
        """Fills the DVL_Data instance with usually random values."""
        self.dvl_data.is_valid = self.rand.choice((True, False))
        self.dvl_data.count = self.rand.randint(0, 255) # From the collected example data it's always 0
        self.dvl_data.struct_id = self.rand.randint(0, 255) # From the collected example data it's always 170
        self.dvl_data.version = 17 # From collected example data
        self.dvl_data.system_type = 76 # From wayfinder documentation
        self.dvl_data.system_subtype = 0 # From wayfinder documentation
        self.dvl_data.fw_major_version = 1
        self.dvl_data.fw_minor_version = 0
        self.dvl_data.fw_patch_version = 0
        self.dvl_data.fw_build_version = 46
        self.dvl_data.year = datetime.now().year
        self.dvl_data.month = datetime.now().month
        self.dvl_data.day = datetime.now().day
        self.dvl_data.hour = datetime.now().hour
        self.dvl_data.minute = datetime.now().minute
        self.dvl_data.second = datetime.now().second
        self.dvl_data.coordinate_system = self.rand.randint(0, 3) # From Wayfinder documentation
        self.dvl_data.vel_x = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.vel_y = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.vel_z = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.range_beam1 = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.range_beam2 = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.range_beam3 = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.range_beam4 = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.mean_range = self.rand.choice((self.rand.uniform(0, 10000), nan))
        self.dvl_data.speed_of_sound = self.rand.uniform(0, 10000)
        self.dvl_data.status = 255
        self.dvl_data.bit_code = self.rand.randint(0, 255)
        self.dvl_data.bit_count = self.rand.randint(0, 255)
        self.dvl_data.voltage = self.rand.uniform(0, 500)
        self.dvl_data.transmit_voltage = self.rand.uniform(0, 500)
        self.dvl_data.current = self.rand.uniform(0, 500)
        self.dvl_data.serial_number = '001010' # From collected example data

        

        

