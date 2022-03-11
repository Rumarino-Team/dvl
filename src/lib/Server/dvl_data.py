class DVL_DATA:

    def __init__(self, is_valid=None, count=None, struct_id=None,
                version=None, system_type=None, system_subtype=None, fw_major_version=None,
                fw_minor_version=None, fw_patch_version=None, fw_build_version=None,
                year=None, month=None, day=None, hour=None, minute=None, second=None,
                coordinate_system=None, vel_x=None, vel_y=None, vel_z=None, vel_err=None,
                range_beam1=None, range_beam2=None, range_beam3=None, range_beam4=None,
                mean_range=None, speed_of_sound=None, status=None, bit_count=None,
                bit_code=None, voltage=None, transmit_voltage=None, current=None,
                serial_number=None, dvl_dict_data=None) -> None:
        
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
        if dvl_dict_data:
            self.prepare_data(dvl_dict_data)
    
    def prepare_data(self, dvl_dict: dict):
        for key in vars(self).keys():
            if key in dvl_dict:
                setattr(self, key, dvl_dict[key])


    def clear_data(self):
        for key in vars(self).keys():
            setattr(self, key, None)
    
    @staticmethod
    def get_all_var_names_ls():
        return list(vars(DVL_DATA()).keys())


if __name__=='__main__':    
    print('DVL_DATA Test Variable Names:')
    print(DVL_DATA.get_all_var_names_ls())
    

