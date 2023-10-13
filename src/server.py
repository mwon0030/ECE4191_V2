import socket
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import rospy 
import json

# Create a socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
server_address = ('118.138.56.32', 12345)
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

print("Waiting for a connection...")
client_socket, client_address = server_socket.accept()

while True:
    data = client_socket.recv(1024)
    if not data:
        break
    print(f"Received: {data.decode()}")
    response = "Server received your message."
    client_socket.send(response.encode())

client_socket.close()
server_socket.close()