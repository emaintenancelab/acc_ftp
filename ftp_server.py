# multi threaded ftp server
from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import ThreadedFTPServer


def main():
    authorizer = DummyAuthorizer()
    #authorizer.add_user('user', '12345', '.')
    authorizer.add_user("user", "12345", ".", perm="elradfmw")
    authorizer.add_anonymous(".")
    handler = FTPHandler
    handler.authorizer = authorizer
    server = ThreadedFTPServer(('0.0.0.0', 2121), handler)
    server.serve_forever()


if __name__ == "__main__":
    main
