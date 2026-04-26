import yaml
import math

def calculate_center(polygon):
    if not polygon:
        return 0, 0
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return sum(xs) / len(xs), sum(ys) / len(ys)

def process_map(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    
    standoff = 0.8
    
    for r in data['regions']:
        cx, cy = calculate_center(r['polygon'])
        yaw = r['yaw']
        
        # Shift in the opposite direction of the yaw
        # If yaw is the direction the robot SHOULD face, standing off means 
        # moving backwards from the center in that direction.
        nx = cx - standoff * math.cos(yaw)
        ny = cy - standoff * math.sin(yaw)
        
        r['x'] = round(nx, 3)
        r['y'] = round(ny, 3)
        
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

if __name__ == "__main__":
    process_map('/home/jun/learning_ws/src/mobile_manipulator/config/semantic_map.yaml')
