
from crypt import methods
from flask import Flask, request
from flask import jsonify

from dvl_data import DVL_DATA
from dvl_connector import DVLDummyDevice

GET = 'GET'
POST = 'POST'
DELETE = 'DELETE'
PUT = 'PUT'

OK = 200
FORBIDDEN = 403

DUMMY_DVL_DATA = DVL_DATA()
DUMMY_DVL_DEVICE = DVLDummyDevice()
DVL_DEVICE = DUMMY_DVL_DEVICE

app = Flask(__name__)


@app.route('/')
def default():
    return 'Dummy Text'

@app.route('/dvl', methods=(GET, ))
def get_dvl_info():
    if request.method == GET:
        return jsonify(DVL_DEVICE.get_data()), OK
    else:
        return 'Only supports GET requests', FORBIDDEN






