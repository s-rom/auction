import random as rnd

colors = ['green', 'blue', 'red']

def main():

    n_robots = 3
    generate_world_file(n_robots)
    generate_launch_file("/home/sergi/catkin_ws/src/auction/launch/world_files/"+str(n_robots)+"_robots.world", n_robots)


def generate_world_file(n_robots):
    with open("/home/sergi/catkin_ws/src/auction/launch/world_files/base.world",'r') as base_world_file:
        with open("/home/sergi/catkin_ws/src/auction/launch/world_files/"+str(n_robots)+"_robots.world", "w") as world_file:
            world_file.write(base_world_file.read())
            for i in range(0, n_robots):
                world_file.write("\n"+generate_random_robot(i)+"\n")

def generate_launch_file(world_file_name,n_robots):
    with open("/home/sergi/catkin_ws/src/auction/launch/"+str(n_robots)+"_robots.launch",'w') as launch_file:
        launch_file.write("<launch>\n\n")

        # stage node
        launch_file.write("\t<node pkg=\"stage_ros\" type=\"stageros\" name=\"stage\" args=\""+world_file_name+"\"/>\n")

        # Add robots
        for i in range(0, n_robots):
            launch_file.write("\t<include file=\"$(find auction)/launch/launch_1_robot.launch\">\n")
            launch_file.write("\t\t<arg name=\"robot_name\" value=\"robot_"+str(i)+"\"/>\n")
            launch_file.write("\t</include>\n")
    
        launch_file.write("\n</launch>")


def generate_random_robot(id):
    global colors
    x = rnd.randrange(-10,10)
    y = rnd.randrange(-10,10)
    color = colors[rnd.randint(0,len(colors)-1)]
    robot = get_robot_world_chunk(id, x, y, 0, 0, color)
    return robot

def get_robot_world_chunk(robot_id, x, y, z, yaw, color):
    data = ''
    with open("/home/sergi/catkin_ws/src/auction/launch/world_files/pioner.txt") as generic_file:
        data = generic_file.read()
        data = data.replace('@robot_name', 'robot_' + str(robot_id))
        data = data.replace('@yaw', str(yaw))
        data = data.replace('@x', str(x))
        data = data.replace('@y', str(y))
        data = data.replace('@z', str(z))
        data = data.replace('@color', color)
    return data
    

if __name__ == "__main__":
    main()