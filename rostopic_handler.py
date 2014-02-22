from MassLib import mk_process

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import dataflow

class rostopic_handler():
    def __init__(self, name):
        self.name = name
        self.get_info()
        self.import_msg()
        self.data = None
        self.publisher = rospy.Publisher(self.name, self.msg_type)
        self.subscriber = rospy.Subscriber(self.name, self.msg_type, self.data_callback) 
        rospy.sleep(1)
    def get_info(self):
        self.info = mk_process('rostopic info %s' % self.name, get_output=True, quiet=True).split('\n')
        self.type = self.info[0].split(': ')[1]

        if 'Subscribers: ' in self.info:
            self.sub_index = self.info.index('Subscribers: ')
            self.subscribers = [sub.split('* ')[1] for sub in self.info[self.sub_index+1:] if sub != '']
        else:
            self.sub_index = self.info.index('Subscribers: None')
            self.subscribers = []
        if 'Publishers: ' in self.info:
            self.pub_index = self.info.index('Publishers: ')
            self.publishers = [pub.split('* ')[1] for pub in self.info[self.pub_index+1:self.sub_index-1] if pub != '']
        else:
            self.pub_index = self.info.index('Publishers: None')
            self.publishers = []

    def import_msg(self):
        msg_group = self.type.split('/')[0]
        msg_type = self.type.split('/')[1]
        if msg_type not in locals():
            exec('from %s.msg import %s' % (msg_group, msg_type))
        self.msg_type = locals()[msg_type]

    def data_callback(self, data):
        self.data = data

    def pub(self, data):
        self.publisher.publish(data)

    def wait(self, timer=5.0):
        dataflow.wait_for(
            lambda: self.data != None,
            timeout=timer
        )

if __name__=='__main__':
    rospy.init_node('rostopic_tester')
    #topic = rostopic_handler('/cameras/left_hand_camera/camera_info')
    halo_g = rostopic_handler('/robot/sonar/head_sonar/lights/set_green_level')
    halo_g.pub(100.0)
    rospy.sleep(1)
