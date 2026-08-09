import re

path = "urdf/assembly_2.urdf"
with open(path, "r", encoding="utf-8") as f:
    text = f.read()

visual_re = re.compile(
    r'(?P<indent>[ \t]*)<visual>\n'
    r'(?P<indent2>[ \t]*)<origin xyz="(?P<xyz>[^"]*)" rpy="(?P<rpy>[^"]*)" />\n'
    r'[ \t]*<geometry>\n'
    r'[ \t]*<mesh filename="(?P<filename>[^"]*)" scale="(?P<scale>[^"]*)" />\n'
    r'[ \t]*</geometry>\n'
    r'(?:[ \t]*<material[^>]*>\n[ \t]*<color[^/]*/>\n[ \t]*</material>\n)?'
    r'(?P=indent)</visual>\n'
)

count = 0

def replacer(m):
    global count
    count += 1
    indent = m.group("indent")
    indent2 = m.group("indent2")
    collision = (
        f'{indent}<collision>\n'
        f'{indent2}<origin xyz="{m.group("xyz")}" rpy="{m.group("rpy")}" />\n'
        f'{indent2}<geometry>\n'
        f'{indent2}    <mesh filename="{m.group("filename")}" scale="{m.group("scale")}" />\n'
        f'{indent2}</geometry>\n'
        f'{indent}</collision>\n'
    )
    return m.group(0) + collision

new_text, n = visual_re.subn(replacer, text)
print("collision 태그 삽입 개수:", n)

with open(path, "w", encoding="utf-8") as f:
    f.write(new_text)