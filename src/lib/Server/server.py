from flask import Flask, request
from flask import jsonify
from os import system
from sys import platform

# Used for text color in terminal
ANSI_YELLOW = '\033[93m'
ANSI_ESC = '\033[0m'

try:
    from dvl_data import DVL_DATA
    from dvl_connector import DVLDummyDevice, WayfinderDVL
except ImportError:
    if platform == 'win32':
        system("") # Enables windows color characters in terminal
    print(ANSI_YELLOW +'\nTeledyne Wayfinder library not installed.\nTry running the command:\npip3 install /src/dvl/src/lib/Wayfinder\n' + ANSI_ESC)
    raise ImportError

# HTTP Methods
GET = 'GET'
POST = 'POST'
DELETE = 'DELETE'
PUT = 'PUT'

# HTTP request responses
OK = 200
FORBIDDEN = 403

# DVL device object creation, init both devices for ease of use
DUMMY_DVL_DATA = DVL_DATA()
DUMMY_DVL_DEVICE = DVLDummyDevice() 
WAYFINDER_DVL = WayfinderDVL()


DVL_DEVICE = WAYFINDER_DVL
DVL_DEVICE = DUMMY_DVL_DEVICE # Always set DVL_DEVICE as dummy, comment this line to use actual wayfinder dvl
if not DVL_DEVICE.connect():
    print('Connection to DVL failed, program will exit...')
    exit()

app = Flask(__name__)

@app.route('/')
def default():
    '''Default route, returns dummy text. Not used'''
    return 'Dummy Text'

@app.route('/dvl', methods=(GET,))
def get_dvl_info():
    '''
    Method called for route /dvl
    
    RETURN: HTTP response with dvl device data in json format
    '''
    if request.method == GET:
        return jsonify(DVL_DEVICE.get_data()), OK
    else:
        return 'Only supports GET requests', FORBIDDEN

