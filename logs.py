import logging
import logging.handlers
import os

home = os.environ.get("HOME")
user = os.environ.get("USER")

LOG_FILENAME = "%s/logging.out" % home

class logger(object):
    def __init__(self, LOG_FILENAME):
        self.user = os.environ.get("USER")
        
        self.logger = logging.getLogger("MyLogger")
        self.user = {'user': user}
        if not len(self.logger.handlers):
            self.logger.setLevel(logging.DEBUG)
        
            self.handler = logging.handlers.RotatingFileHandler(LOG_FILENAME,
                                                       maxBytes=200000,
                                                       backupCount=5,
                                                       )
            self.logger.addHandler(self.handler)

            self.formatter=logging.Formatter("%(asctime)s %(user)s ~ %(message)s", 
                                         "%m/%d/%Y %H:%M:%S")
            self.handler.setFormatter(self.formatter)
        
    def debug(self, message):
        self.logger.debug(message, extra=self.user)
        
def main():
    logger = logger(LOG_FILENAME)

    logger.debug("hello there")

if __name__ == "__main__":
    main()
