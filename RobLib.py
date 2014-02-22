from MassLib import mk_process, mkdirs, timeout
from ssh import Ssh
from rostopic_handler import rostopic_handler
from socket import socket, AF_INET, SOCK_STREAM, gethostbyname
import time
import os

class Robot(object):
    def __init__(self, hostname, hlr_pass, root_pass, logger):
        self.robot = hostname
        self.hlr_pass = hlr_pass
        self.root_pass = root_pass
        self.create_clients()
        self.robotIP=gethostbyname(self.robot)
        self.joint_states = rostopic_handler('/robot/joint_states')
        self.left_arm = self.limb(self, 'left')
        self.right_arm = self.limb(self, 'right')
        self.left_arm_speed = rostopic_handler('/robot/limb/left/set_speed_ratio')
        self.right_arm_speed = rostopic_handler('/robot/limb/right/set_speed_ratio')
        self.logger = logger

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
    Reboot the robot, close current ssh clients, and create new ones
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''   
    def reboot(self):
        print 'Rebooting the robot now'
        self.root_client.exec_command('reboot', quiet=True)
        self.close()
            # Ping the robot until ping is unsuccessfull (indicating that the robot is now off)
        while True:
            if self.is_on():
                break
                
        # Continue pinging the robot until ping is successfull (indicating that the robot is now on)
        print "Robot is off"
        while True:
            if not self.is_on():
                break
        print "Robot is now on"
        
        # Attempt to open socket to robot at port 22 until successful (indicating that the SSH port is now open)
        while True:
            if self.has_ssh():
                break
        return self.check_online() 
        
    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Verify that all rc's have started
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''   
    def check_online(self):
        print "Waiting until all RCs are started"
        start = time.time()
        while True:
            if time.time()-start > 45:
                print 'RCs did not all start'
                return False
            if self.root_client.return_command('rc-status | grep -i starting') == [] and self.root_client.return_command('rc-status | grep -i stopped') == []:
                print 'rebooted\n'
                return True

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Delete files using the delete command in the Ssh object.
    Uses a "rm -rf" command
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~''' 
    def delete(self, files):
        self.root_client.delete(files)
    
    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Create ssh clients
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''     
    def create_clients(self):
        self.hlr_client = Ssh('hlr', self.robot, self.hlr_pass)
        self.root_client = Ssh('root', self.robot, self.root_pass)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Close ssh clients
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''     
    def close_clients(self):
        self.hlr_client.close()
        self.root_client.close()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Returns a bool indicating whether or not the robot is responding to pings.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def is_on(self):
        return mk_process('ping %s -c 1 -w 1' % self.robot, quiet=True)==1

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Returns a bool indicating whether or not ssh is running on the robot.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def has_ssh(self):
        s = socket(AF_INET, SOCK_STREAM)
        result = s.connect_ex((self.robotIP, 22))
        s.close()
        if(result == 0) :
            print 'SSH port open'
            return True
        else:
            return False

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    SCP a simple C file to the robot that, when run, will generate a core file
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def send_crash(self):
        self.hlr_client.scp_put('tester/testbody/resources/crash', '.')
    
    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Runs a simple C file on the robot to generate a core file.
    This function should be called only after send_crash.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def gen_core(self):
        self.hlr_client.exec_command('./crash')

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Deletes the crash file sent by send_crash()
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def delete_crash(self):
        self.hlr_client.delete('crash')

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Starts the given service on the robot
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def start_service(self, service):
        self.root_client.exec_command('/etc/init.d/%s start' % service)
        if service=='hlr-sdk':
            self.root_client.exec_command('killall X; rc-service fsm-rsdk restart')


    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Stops the given service on the robot
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def stop_service(self, service):
        self.root_client.exec_command('/etc/init.d/%s stop' % service)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Returns that status of the given service on the robot
    If an invalid service, will return False.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def service_status(self, service):
        status = self.root_client.return_command('/etc/init.d/%s status' % service)
        if len(status):
            return status[0].split()[2]
        else:
            return False
    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Enables the robot with the enable_robot example program
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def enable(self):
        mk_process('rosrun tools enable_robot.py -e')

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Returns the status of the robot as shown by the
    enable_robot example program when run with the -s argument
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def get_status(self):
        return mk_process('rosrun tools enable_robot.py -s', get_output=True)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Resets the image on the robot to the default BRR background image
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def reset_image(self):
        self.logger.debug('sending default image')
        self.logger.debug(mk_process('ls', get_output=True))
        mk_process('rosrun head_control xdisplay_image.py -f tester/testbody/resources/researchsdk.png')
    
    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Verify that the robot is on and functioning.
    Used to verify the success of stress tests.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
    def check_on(self):
        # Is the robot on?
        if mk_process('ping %c -c 1 -w 1' % self.robot, quiet=True)!=1:
            return [False, 'power_failure']
        if mk_process('rostopic list', quiet=True, get_output=True).startswith('ERROR: Unable to communicate with master!'):
            return [False, 'roscore_failure']
        if 'stopped' in self.root_client.return_command('rc-status | grep hlr-sdk')[0]:        
            return [False, 'hlr-sdk_failure']
        return [True, 'robot working']

    class limb():
        def __init__(self, parent, side):
            self.parent = parent
            self.side = side
            self.j_cmd = rostopic_handler('/robot/limb/%s/joint_command' % self.side)
            self.j_msg = self.j_cmd.msg_type()
            self.j_msg.mode = self.j_cmd.msg_type.POSITION_MODE

        def get_position(self, joint):
            return self.parent.joint_states.data.position[self.parent.joint_states.data.name.index(joint)]

        def set_position(self, joint, value):
            self.j_msg.names = [joint]
            self.j_msg.command = [value]
            while abs(self.get_position(joint)-value)>.02:
                self.j_cmd.pub(self.j_msg)
            

def get_robot():
    rob_inst = os.environ.get('HLR_ROBOT_INSTANCE')
    print "Robot Hostname: \"%s\"" % rob_inst
    return rob_inst
