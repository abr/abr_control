# Echo client program: turn DO 2 on
"""import socket
HOST = "192.168.0.9"    # The remote host
PORT = 30002              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send (b"set_digital_out(2,True)" + b"\n")
data = s.recv(1024)
s.close()
print ("Received", repr(data))"""

# Echo client program: turn DO 2 off
import socket
HOST = "192.168.0.9"    # The remote host
PORT = 30002              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send (b"set_digital_out(2,False)" + b"\n")
data = s.recv(1024)
s.close()
print ("Received", repr(data))
