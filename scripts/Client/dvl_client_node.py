#!/usr/bin/env python

from time import sleep
from std_msgs.msg import Header
from dvl.msg import Raw_DVL
from rospy import Publisher, Rate, init_node, is_shutdown, Time
from requests import get
from requests.exceptions import ConnectionError

class DVLNodePublisher:

    URL_ROUTE = 'http://127.0.0.1:5000/dvl' # Default url route to get dvl information, should never change

    """Used when the node is running, if the http server is not running after max requests amount to server is reached it will wait by this value to try again."""
    DEFAULT_SLEEP_TIMEOUT = 3

    def __init__(self, topic_to_publish_to, node_base_name, rate=10, default_queue_size=10):
        """
        Creates DVLNodePublisher instance

        Args:
            topic_to_publish: str -- Topic name the ros publisher will be publishing to.
            node_base_name: str -- Name of the ros node.
            rate: int (default 10) -- Rate at which the ros node will operate on in amount of Hz.
            default_queue_size: int (default 10) -- Limits the amount of messages in the queue if a subscriber is not fast enough to receive them.

        Returns:
            Instance of DVLNodePublisher

        """
        self.dvl_msg_data = Raw_DVL() # Create message instance, instance will be reused by node to provide information
        self.pub = Publisher(topic_to_publish_to, Raw_DVL, queue_size=default_queue_size) # Create ros publisher to given arguments
        init_node(node_base_name, anonymous=True) # Init ros node
        self.rate = Rate(rate) # Set rate for ros node

    def publish_dvl_message(self):
        """Requests data from http server and publishes the message to ros, if the request fails it waits for the amount specified rate."""
        if self._request_dvl_data():            
            self.dvl_msg_data.header = self._get_msg_header() # Sets messageheader to message
            self.pub.publish(self.dvl_msg_data) # Message is published to ros with publisher
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
        """
        Creates a header instance to be used for the message that is going to be published.

        Returns:
            msg_header -- ros Header message instance

        Raises:
            ROSException: ros must be initialized first, usually with init method from rospy
        """
        msg_header = Header()
        msg_header.stamp = Time.now()
        return msg_header
        
            


if __name__=='__main__': # Code runs if this script is loaded as main
    dvl_pub = DVLNodePublisher('dvl', 'Wayfinder_DVL_Filter') # Creates instance of DVLNodePublisher
    while not is_shutdown(): # Keep working while ROS on
        try:
            dvl_pub.publish_dvl_message() # Publish dvl message to ros
        except ConnectionError: # If the server is not running or having errors this exception is raised and node will wait until it tries again
            print 'Could not reach http server, retrying in %ss...' % DVLNodePublisher.DEFAULT_SLEEP_TIMEOUT
            sleep(DVLNodePublisher.DEFAULT_SLEEP_TIMEOUT)
        


