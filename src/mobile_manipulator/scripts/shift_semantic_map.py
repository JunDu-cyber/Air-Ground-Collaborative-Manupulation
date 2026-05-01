import yaml
import argparse
import math

def shift(dx, dy):
    with open('config/semantic_map.yaml') as f:
        data = yaml.safe_load(f)
    for r in data['regions']:
        r['x'] = float(r['x'] + dx)
        r['y'] = float(r['y'] + dy)
        r['polygon'] = [[float(v[0]+dx), float(v[1]+dy)] for v in r['polygon']]
    with open('config/semantic_map.yaml', 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print("Done")

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dx', type=float, required=True, help='Translation in x (world metres)')
    p.add_argument('--dy', type=float, required=True, help='Translation in y (world metres)')
    args = p.parse_args()
    shift(args.dx, args.dy)

if __name__ == "__main__":
    main()