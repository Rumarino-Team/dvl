#!/usr/bin/env python

from time import sleep
from std_msgs.msg import Header
# from dvl_pub_pkg.msg import Wayfinder_DVL
from dvl.msg import Raw_DVL
from rospy import Publisher, Rate, init_node, is_shutdown, Time
from requests import get
from requests.exceptions import ConnectionError

DEFAULT_WAIT_TIME_FOR_RETRY = 3


class DVLNodePublisher:

    URL_ROUTE = 'http://127.0.0.1:5000/dvl'
    DEFAULT_SLEEP_TIMEOUT = 3

    def __init__(self, topic_to_publish_to, node_base_name, rate=10, default_queue_size=10):
        self.dvl_msg_data = Raw_DVL()        
        self.pub = Publisher(topic_to_publish_to, Raw_DVL, queue_size=default_queue_size)
        init_node(node_base_name, anonymous=True)
        self.rate = Rate(rate)

    def publish_dvl_message(self):
        if self._request_dvl_data():            
            self.dvl_msg_data.header = self._get_msg_header()
            self.pub.publish(self.dvl_msg_data)        
        self.rate.sleep()

    def _request_dvl_data(self):
        server_response = get(DVLNodePublisher.URL_ROUTE)
        if server_response.ok:
            self._populate_dvl_msg_data(server_response.json()['DVL_Data'])
            return True
        else:
            print 'Could not retrieve data from DVL server.'
            return False
            
    
    def _populate_dvl_msg_data(self, data_from_request):        
        for key, val in data_from_request.items(): 
            if key in dir(self.dvl_msg_data):                            
                if isinstance(val, unicode):
                    msg_attr = getattr(self.dvl_msg_data, key)
                    msg_attr.data = val.encode('utf-8')                        
                else:          
                    setattr(self.dvl_msg_data, key, val)

    def _get_msg_header(self):
        msg_header = Header()
        msg_header.stamp = Time.now()
        return msg_header
        
            


if __name__=='__main__':
    dvl_pub = DVLNodePublisher('dvl', 'Wayfinder_DVL_Filter')
    while not is_shutdown():
        try:
            dvl_pub.publish_dvl_message()
        except ConnectionError:
            print 'Could not reach http server, retrying in %ss...' % DEFAULT_WAIT_TIME_FOR_RETRY
            sleep(DEFAULT_WAIT_TIME_FOR_RETRY)
        


