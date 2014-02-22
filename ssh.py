import paramiko
import pexpect

class Ssh:
    '''
    This class handles basic SSH and SCP functionality, using paramiko and pexpect.
    It is initialized with a username, hostname, and password.
    It exposes an exec_command, return_command (for getting back the data from a command),
        scp_get, and scp_put.
    '''
    RET_SUCCESS = 0
    RET_FAIL    = 1
    RET_TIMEOUT = 2
    RET_EOF     = 3
    RET_ERROR   = 5

    def __init__ (self, p_username, p_hostname, p_password):
        self._username = p_username
        self._hostname = p_hostname
        self._password = p_password
        self._client = paramiko.SSHClient()
        self._create_ssh_client()
        self._expect_list = [
            pexpect.EOF,
            'Permission denied',
            "%s@%s's password" % (self._username, self._hostname),
            'Are you sure you want to continue connecting (yes/no)?',
            'Password:',
        ]

    ## Takes bash command and executes on remote machine.
    #  @param quiet Tells the function not to print the output of the command
    def exec_command(self, command, quiet=True):
        (stdin, stdout, stderr) = self._client.exec_command(command)
        data = stdout.readlines()
        if quiet == False:
            print data
        data = stderr.readlines()
        if quiet == False:
            print data

    ## Returns the output of a command run on an external machine
    def return_command(self, command):
        (stdin, stdout, stderr) = self._client.exec_command(command)
        data = stdout.readlines()
        return data

    ## Takes hostname, username, and password as arguments,
    #  and creates an SSHCLient object to be used by ssh and scp scripts
    def _create_ssh_client(self):
        self._client.load_system_host_keys()
        self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        while 1:
            self._client.connect(self._hostname, 22, self._username, self._password)
            trans = self._client.get_transport()
            if not trans.get_exception():
                break
    def scp_get(self, init_dir, fin_dir, recursive=False):
        rcsv = ('', ' -r')[recursive]
        cmd = 'scp%s %s@%s:%s %s' % (rcsv, self._username, self._hostname,
                                     init_dir, fin_dir)

        self._run_scp(cmd)

    def scp_put(self, init_dir, fin_dir, recursive=False):
        rcsv = ('', ' -r')[recursive]
        cmd = 'scp%s %s %s@%s:%s' % (rcsv, init_dir,
                                     self._username, self._hostname, fin_dir)
        self._run_scp(cmd)

    def _run_scp(self, cmd):
        self.child = pexpect.spawn(cmd, [], timeout=125)

        while True:
            idx = self.child.expect(self._expect_list)

            if idx == 0:
                return self.RET_SUCCESS

            elif idx == 1:
                return self.RET_FAIL

            elif idx == 2:
                self.child.sendline(self._password)

            elif idx == 3:
                self.child.sendline('yes')

            elif idx == 4:
                self.child.sendline(self._password)

    def delete(self, files):
        cmd = 'rm -rf %s' % files
        self.exec_command(cmd, quiet=True)

    ## Closes ssh client session
    def close(self):
        self._client.close()


if __name__ == '__main__':
    pass

