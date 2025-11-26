import getpass, pprint


name = f'/home/{getpass.getuser()}/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'

with open(name, 'r+') as file:
    offsets = []
    offset = 0
    contents = ''
    for index, content in enumerate(file.readlines()):
        if '<include>' in content:
            break
        offsets.append(offset)
        offset += len(content)
        contents += content
    saved_index = index
    contents += '\n'
    contents += '''    <model name="carpet">
        <static>true</static>
        <link name="square_link">
            <pose>1 1 0.05 0 0 0.785398</pose>
            <visual name="square_texture">
                <cast_shadows>false</cast_shadows>
                <geometry>
                    <box>
                        <size>4.0 0.5 1e-3</size>
                    </box>
                </geometry>
                <material>
                    <script>
                        <uri>model://square_line/materials/scripts</uri>
                        <uri>model://square_line/materials/textures</uri>
                        <name>square_solid</name>
                    </script>
                </material>
            </visual>
        </link>
    </model>'''
    contents += '\n'
with open(name, 'r+') as file:
    for index, content in enumerate(file.readlines()[saved_index::]):
        offsets.append(offset)
        offset += len(content)
        contents += content

with open(name, 'w') as file:
    file.write(contents)
    pprint.pprint(contents)