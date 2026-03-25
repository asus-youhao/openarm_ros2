import xml.etree.ElementTree as ET
import os

# Define the path to the URDF file
fn = '/tmp/robot.urdf'

# Check if the file exists before processing
if not os.path.exists(fn):
    print(f"Error: URDF file not found at {fn}")
    exit()

# Parse the XML structure of the URDF
tree = ET.parse(fn)
root = tree.getroot()

# 1. Collect all link names defined in the URDF
links = [l.attrib['name'] for l in root.findall('link')]

# 2. Build mapping for parent-child relationships
# parent_map[child_name] = (parent_name, joint_name)
parent_map = {}
joints = root.findall('joint')

for j in joints:
    jname = j.attrib.get('name', 'unnamed_joint')
    parent = j.find('parent').attrib['link']
    child = j.find('child').attrib['link']
    parent_map[child] = (parent, jname)

# --- Output Section ---

print(f'Total links: {len(links)}')
print('Sample links (first 40):')
for L in links[:40]:
    print('  ', L)

print('\nLooking for links that end with "link7"...')
targets = [ln for ln in links if ln.endswith('link7')]

if not targets:
    print('  No link matching "*link7" found.')
else:
    for t in targets:
        print(f'\nChain to {t} :')
        
        # Trace the path backwards from the target link to the root (e.g., world)
        path = []
        joint_stack = []
        curr = t
        
        while curr in parent_map:
            p, j = parent_map[curr]
            path.append(curr)
            joint_stack.append(j)
            curr = p
        
        # Append the top-most ancestor (usually 'world')
        path.append(curr)
        
        # Reverse the lists to display the hierarchy from root to leaf
        path.reverse()
        joint_stack.reverse()
        
        print(f'  root -> {path[0]}')
        for node in path[1:]:
            print(f'   -> {node}')
            
        print('Joints on path:')
        for j in joint_stack:
            print(f'   {j}')

# 3. Check for the existence of <inertial> tags in specific links
# This is crucial for physics simulations (like Gazebo or PyBullet)
print('\nChecking for <inertial> presence in links that contain "hand" or "finger":')
for link_node in root.findall('link'):
    lname = link_node.attrib['name']
    if 'hand' in lname or 'finger' in lname:
        inert = link_node.find('inertial')
        # Check if the inertial tag exists under the current link
        status = 'YES' if inert is not None else 'NO'
        print(f'   {lname} -> inertial: {status}')

# 4. DEBUG: Check connection between right arm and hand
print('\n--- DEBUG: Checking right_palm_lower connection ---')
if 'right_palm_lower' in parent_map:
    parent, joint = parent_map['right_palm_lower']
    print(f'right_palm_lower is connected via joint "{joint}" to parent "{parent}"')
else:
    print('ERROR: right_palm_lower has NO parent! (orphaned link)')
    
# Find all joints that connect to openarm_right_link7
print('\n--- Joints with openarm_right_link7 as parent ---')
for j in joints:
    parent_link = j.find('parent').attrib['link']
    child_link = j.find('child').attrib['link']
    jname = j.attrib.get('name', 'unnamed')
    if parent_link == 'openarm_right_link7':
        print(f'  Joint: {jname}')
        print(f'    parent: {parent_link} -> child: {child_link}')

if not any(j.find('parent').attrib['link'] == 'openarm_right_link7' for j in joints):
    print('  No joints found with openarm_right_link7 as parent!')
    print('  This means the hand is NOT connected to the arm.')
    
# List all hand-related links
print('\n--- All hand/palm related links ---')
hand_links = [l for l in links if 'palm' in l or 'hand' in l]
for hl in hand_links:
    if hl in parent_map:
        p, j = parent_map[hl]
        print(f'  {hl} <- (via {j}) <- {p}')
    else:
        print(f'  {hl} <- NO PARENT (orphaned!)')