import re

PATH = "urdf/new_robot.urdf"

with open(PATH, encoding="utf-8") as f:
    text = f.read()

# 1. add <collision> mirroring every <visual> mesh block
visual_re = re.compile(
    r'(?P<indent>[ \t]*)<visual>\n'
    r'(?P<indent2>[ \t]*)<origin xyz="(?P<xyz>[^"]*)" rpy="(?P<rpy>[^"]*)" />\n'
    r'[ \t]*<geometry>\n'
    r'[ \t]*<mesh filename="(?P<filename>[^"]*)" scale="(?P<scale>[^"]*)" />\n'
    r'[ \t]*</geometry>\n'
    r'(?:[ \t]*<material[^>]*>\n[ \t]*<color[^/]*/>\n[ \t]*</material>\n)?'
    r'(?P=indent)</visual>\n'
)

collision_count = 0

def add_collision(m):
    global collision_count
    collision_count += 1
    indent, indent2 = m.group("indent"), m.group("indent2")
    collision = (
        f'{indent}<collision>\n'
        f'{indent2}<origin xyz="{m.group("xyz")}" rpy="{m.group("rpy")}" />\n'
        f'{indent2}<geometry>\n'
        f'{indent2}    <mesh filename="{m.group("filename")}" scale="{m.group("scale")}" />\n'
        f'{indent2}</geometry>\n'
        f'{indent}</collision>\n'
    )
    return m.group(0) + collision

text, n = visual_re.subn(add_collision, text)
print(f"collision blocks added: {n}")

# 2. revolute_1 / revolute_2 (M3 screw-thread mates) should not spin freely
for jname in ("revolute_1", "revolute_2"):
    pattern = f'<joint name="{jname}" type="continuous">'
    replacement = f'<joint name="{jname}" type="fixed">'
    if pattern not in text:
        raise SystemExit(f"could not find {pattern!r}")
    text = text.replace(pattern, replacement, 1)
    print(f"{jname}: continuous -> fixed")

# 3. flip axis sign on the right-side wheel joints so both sides share the
#    same world-frame spin direction (otherwise the DiffDrive plugin spins
#    the vehicle in place instead of driving straight)
axis_fixes = {
    "revolute_5": ('<axis xyz="-0 -0 1" />', '<axis xyz="0 0 -1" />'),
    "revolute_6": ('<axis xyz="0 0 -1" />', '<axis xyz="0 0 1" />'),
}
for jname, (old_axis, new_axis) in axis_fixes.items():
    block_re = re.compile(
        rf'(<joint name="{jname}" type="continuous">\s*<origin[^/]*/>\s*)'
        rf'{re.escape(old_axis)}'
    )
    text, n = block_re.subn(lambda m: m.group(1) + new_axis, text, count=1)
    if n != 1:
        raise SystemExit(f"could not patch axis for {jname}")
    print(f"{jname}: axis flipped ({old_axis} -> {new_axis})")

# 4. DiffDrive plugin, right after <robot name="new_robot">
plugin_block = '''<robot name="new_robot">
    <gazebo>
        <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
            <left_joint>revolute_3</left_joint>
            <left_joint>revolute_4</left_joint>
            <right_joint>revolute_5</right_joint>
            <right_joint>revolute_6</right_joint>
            <wheel_separation>0.5201</wheel_separation>
            <wheel_radius>0.105</wheel_radius>
            <odom_publish_frequency>20</odom_publish_frequency>
            <topic>cmd_vel</topic>
            <frame_id>odom</frame_id>
            <child_frame_id>root</child_frame_id>
        </plugin>
    </gazebo>
'''
if '<robot name="new_robot">\n' not in text:
    raise SystemExit("could not find robot root tag")
text = text.replace('<robot name="new_robot">\n', plugin_block, 1)
print("DiffDrive plugin inserted")

with open(PATH, "w", encoding="utf-8") as f:
    f.write(text)

print("done")