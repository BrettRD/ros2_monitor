import socket
from launch import LaunchDescription
from launch_ros.actions import Node

machine_hostname = socket.gethostname()
machine_hostname = machine_hostname.replace('-', '_')

def generate_launch_description():
  nodes = [
    Node(
      name='monitor_node',
      namespace=machine_hostname,
      package='monitoring',
      executable='monitor_node',
      remappings=[
        ('temperature', 'temperature'),
      ],
      #prefix=['xterm -e gdb -ex run --args'],
      parameters=[{
        'rate':1000,    # not implemented
      }],
    ),
  ]
  return LaunchDescription(nodes)
