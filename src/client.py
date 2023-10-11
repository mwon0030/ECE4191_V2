import socket
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import rospy 
import json

SERVER_IP = '118.138.56.32'
PORT_NUMBER = 12345

x = 0
y = 0
th = 0
obstacle_detected = False
goal_location = []

message_to_send_to_server = {}\

def state_cb(data):
    x = data.data[0]
    y = data.data[1]
    th = data.data[2] 

def obstacle_detect_cb(data):
    obstacle_detected = data.data

def goal_location_cb(data):
    goal_location = data.data

state_pub = rospy.Subscriber('/state', Float32MultiArray, state_cb)
obstacle_detect_sub = rospy.Subscriber('/obstacle_detect', Bool, obstacle_detect_cb)
goal_location_sub = rospy.Subscriber('/goal_location', Bool, goal_location_cb)


# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
server_address = (SERVER_IP, PORT_NUMBER)
client_socket.connect(server_address)

while not rospy.is_shutdown():
    try:
        message_to_send_to_server['x'] = x
        message_to_send_to_server['y'] = y
        message_to_send_to_server['th'] = th
        message_to_send_to_server['obstacle_detected'] = obstacle_detected
        message_to_send_to_server['goal_location'] = goal_location

        client_socket.send(json.dumps(message_to_send_to_server).encode())
        data = client_socket.recv(1024)
        print(f"Server response: {data.decode()}")

    except rospy.ROSInterruptException:
        client_socket.close() 
        break

