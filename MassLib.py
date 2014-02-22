import os
from subprocess import Popen, PIPE, STDOUT
import signal

class TimeoutException(Exception):
    pass

#@ Creates a python decorator to add a timeout to a function.
#  Displays the "default" message if the timeout
#  is reached before a return on the funciton.
#  Returns a function that will attempt to run the decorated function but will
#  return the "default" value if nothing is returned in "timeout_time" seconds.
def timeout(timeout_time, default):
    def timeout_function(f):
        def f2(*args):
            def timeout_handler(signum, frame):
                raise TimeoutException()

            old_handler = signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(timeout_time)
            try:
                retval = f(*args)
            except TimeoutException:
                return default
            finally:
                signal.signal(signal.SIGALRM, old_handler)
            signal.alarm(0)
            return retval
        return f2
    return timeout_function

## Simple function to execute a linux shell command
#  @param command The bash command to be run
#  @param quiet Tells the function no to print the output of the command
#  @param get_output Tells the script to return the output of the command
def mk_process(command, quiet=False, get_output=False):
    process = Popen(command, shell = True, stdout = PIPE,
                               stderr = STDOUT)
    stdout, stderr = process.communicate()
    if quiet == False:
        print stdout
    if get_output:
        return stdout
    else:
        return process.returncode

## Equivalent of "mkdir -p" in linux shell
def mkdirs(path):
    try:
        os.makedirs(path)
    except:
        print "Folder \"%s\" already exists" % path

class ros_process():
    def __init__(self, command, quiet=True, get_output=False):
        self.process = Popen(command, shell=True, stdout=PIPE,
                                        stdin=PIPE, stderr=STDOUT)
        self.quiet = quiet
        self.get_output = get_output

    def write(self, stdin):
        self.process.stdin.write(stdin)

    def run(self):
        stdout, stderr = self.process.communicate()
        if self.quiet == False:
            print stdout
        if self.get_output:
            return stdout
        else:
            return self.process.returncode

## Example program showing how to use the timeout decorator
@timeout(3, "You didn't type anything")
def do_stuff():
    raw_input("Type something, or don't...:__________\n")
    return ""

if __name__ == "__main__":
    print do_stuff()
