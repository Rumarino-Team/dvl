class DVL_DATA:
    """
    Data class that will contain the information from the DVL device and then be used to be sent via HTTP get request.
    !Important! The argument dvl_data_dict can be used to supply a dictionary with all the data (key, val) for the data class and
    it will be assigned automatically withouth having to assign every attribute one by one in the constructor. !See Usage!
    More information about dvl driver: https://teledynerdi.myshopify.com/pages/wayfinder-driver-index

    Args: 
        is_valid: bool
        count: int
        struct_id: int
        version: int
        system_type: int
        system_subtype: int
        fw_major_version: int
        fw_minor_version: int
        fw_patch_version: int
        fw_build_version: int
        year: int 
        month: int
        day: int 
        hour: int
        minute: int
        second: int
        coordinate_system: int
        vel_x: float
        vel_y: float
        vel_z: float
        vel_err: float = vel_err # Beam 4 in m/s
        range_beam1: float
        range_beam2: float
        range_beam3: float
        range_beam4: float
        mean_range: float 
        speed_of_sound: float
        status: int 
        bit_count: int
        bit_code: int
        voltage: float
        transmit_voltage: float
        current: float
        serial_number: str
        #For more information on variables see: https://teledynerdi.myshopify.com/pages/wayfinder-driver-system.html#dvl.system.OutputData

        dvl_data_dict: dict

    Usage:        
        These two DVL_Data instances will be exacly the same after creation:
            dvl_d = DVL_Data(day=1, hour=2, minute=4, ....) # Imagine the long list of arguments
        and 
            some_dic = {'day': 1, 'hour' : 2, 'minute' : 4, ......} # Imagine dictionary contains all data pairs
            dvl_d = DVL_Data(dvl_data_dict=some_dic) # Only one key argument used in constructor
        
        If every keyword argument is provided in the constructor along with the dvl_data_dict, the dictionary will overwrite all previous values with its data.


    Returns:
        DVL_Data instance
    """

    def __init__(self, is_valid=None, count=None, struct_id=None,
                version=None, system_type=None, system_subtype=None, fw_major_version=None,
                fw_minor_version=None, fw_patch_version=None, fw_build_version=None,
                year=None, month=None, day=None, hour=None, minute=None, second=None,
                coordinate_system=None, vel_x=None, vel_y=None, vel_z=None, vel_err=None,
                range_beam1=None, range_beam2=None, range_beam3=None, range_beam4=None,
                mean_range=None, speed_of_sound=None, status=None, bit_count=None,
                bit_code=None, voltage=None, transmit_voltage=None, current=None,
                serial_number=None, displacement={'x':0, 'y':0, 'z':0}, dvl_dict_data=None) -> None:
        
        self.is_valid: bool = is_valid
        self.count: int = count # Count of data packets
        self.struct_id: int = struct_id # Structure id number
        self.version: int = version # Structure version number
        self.system_type: int = system_type # Should be 76 for Wayfinder
        self.system_subtype: int = system_subtype # 0 for Wayfinder
        self.fw_major_version: int = fw_major_version
        self.fw_minor_version: int = fw_minor_version
        self.fw_patch_version: int = fw_patch_version
        self.fw_build_version: int = fw_build_version
        self.year: int = year
        self.month: int = month
        self.day: int = day
        self.hour: int = hour # range(24)
        self.minute: int = minute # range(60)
        self.second: int = second # range(60)
        self.coordinate_system: int = coordinate_system # (0-3)
        self.vel_x: float = vel_x # Beam 1 in m/s
        self.vel_y: float = vel_y # Beam 2 in m/s
        self.vel_z: float = vel_z # Beam 3 in m/s
        self.vel_err: float = vel_err # Beam 4 in m/s
        self.range_beam1: float = range_beam1 # Beam 1 range to bottom in meters
        self.range_beam2: float = range_beam2 # Beam 2 range to bottom in meters
        self.range_beam3: float = range_beam3 # Beam 3 range to bottom in meters
        self.range_beam4: float = range_beam4 # Beam 4 range to bottom in meters
        self.mean_range: float = mean_range # Mean range to bottom in meters
        self.speed_of_sound: float = speed_of_sound # Speed of sound used in m/s
        self.status: int = status
        self.bit_count: int = bit_count
        self.bit_code: int = bit_code
        self.voltage: float = voltage
        self.transmit_voltage: float = transmit_voltage
        self.current: float = current
        self.serial_number: str = serial_number
        self.displacement: list = displacement
        if dvl_dict_data:
            self.prepare_data(dvl_dict_data)
    
    def prepare_data(self, dvl_dict: dict):
        """
        Takes the data given in the dictionary and uses it to set the fields of the current DVL_Data intance.
        Keys must match the key or variable name of the data DVL_Data class, otherwise the value will be ignored.
        
        Args:
            dvl_dict: dict -- dictionary containing data to be assigned to the data class.
        """
        for key in vars(self).keys():
            if key in dvl_dict:
                setattr(self, key, dvl_dict[key])


    def clear_data(self):
        """Delete all instance variable fields in DVL_Data class."""
        for key in vars(self).keys():
            setattr(self, key, None)
        setattr(self, 'displacement', {'x':0, 'y':0, 'z':0})
    
    @staticmethod
    def get_all_var_names_ls():
        """Utility method returns all variable names in a list."""
        return list(vars(DVL_DATA()).keys())


if __name__=='__main__':
    """Testing method from data class."""
    print('DVL_DATA Test Variable Names:')
    print(DVL_DATA.get_all_var_names_ls())
    

