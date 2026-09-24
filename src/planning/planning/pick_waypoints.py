#!/usr/bin/env python3
"""
python3 ~/WORK2-2026/src/planning/planning/pick_waypoints.py ~/WORK2-2026/src/planning/planning/mapa.yaml

world_x = origin_x + px * resolution
world_y = origin_y + (altura_da_imagem - py) * resolution

bibliotecas:
pyyaml = pip install pyyaml --break-system-packages
pillow = pip install pillow --break-system-packages
matplotlib = pip install matplotlib --break-system-packages
"""

import argparse
import os
import sys
import yaml
from PIL import Image
import matplotlib.pyplot as plt

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        map_yaml = yaml.safe_load(f)

    image_field = map_yaml['image']
    image_path = image_field
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_field)

    resolution = float(map_yaml['resolution'])
    origin = map_yaml.get('origin', [0.0, 0.0, 0.0])
    origin_x, origin_y = float(origin[0]), float(origin[1])

    img = Image.open(image_path)
    return img, resolution, origin_x, origin_y, image_path


def pixel_to_world(px, py, img_height, resolution, origin_x, origin_y):
    world_x = origin_x + px * resolution
    world_y = origin_y + (img_height - py) * resolution
    return world_x, world_y


def main():
    parser = argparse.ArgumentParser(description='extrai waypoints de um mapa ROS2/Nav2.')
    parser.add_argument('map_yaml', help='Caminho para o .yaml do mapa (ex: arena.yaml)')
    '''parser.add_argument(
        '-o', '--output', default='locations_output.yaml',
        help='arquivo de saída (default: locations_output.yaml, na pasta atual)')'''
    args = parser.parse_args()

    img, resolution, origin_x, origin_y, image_path = load_map(args.map_yaml)
    img_width, img_height = img.size

    print(f'mapa carregado: {image_path}')
    print(f'  resolução: {resolution} m/px | origem: ({origin_x}, {origin_y}) | '
          f'tamanho: {img_width}x{img_height} px')
    print('clique nos pontos desejados na janela do matplotlib.')
    print('depois de CADA clique, volte pro terminal: ele vai pedir o nome do ponto.')
    print("feche a janela (ou tecle 'q') quando terminar.\n")

    waypoints = {}

    fig, ax = plt.subplots()
    ax.imshow(img, cmap='gray')

    def on_click(event):
        if event.xdata is None or event.ydata is None:
            return  # clique fora da imagem

        px, py = event.xdata, event.ydata
        world_x, world_y = pixel_to_world(px, py, img_height, resolution, origin_x, origin_y)

        print(f'Ponto clicado: pixel=({px:.0f}, {py:.0f}) -> mundo=({world_x:.3f}, {world_y:.3f})')
        name = input('nome do waypoint: ').strip()
        if not name:
            print('pulado.\n')
            return

        theta_str = input('theta em graus: ').strip()
        theta_deg = float(theta_str) if theta_str else 0.0
        theta_rad = theta_deg * 3.141592653589793 / 180.0

        waypoints[name] = (round(world_x, 3), round(world_y, 3), round(theta_rad, 3))

        # marca visualmente o ponto já registrado
        ax.plot(px, py, 'r+', markersize=12, markeredgewidth=2)
        ax.annotate(name, (px, py), color='red', fontsize=9,
                    xytext=(5, 5), textcoords='offset points')
        fig.canvas.draw()
        print(f'  registrado: {name} = [{world_x:.3f}, {world_y:.3f}, {theta_rad:.3f}]\n')

    fig.canvas.mpl_connect('button_press_event', on_click)
    plt.show()

    if not waypoints:
        print('nenhum waypoint registrado.')
        return

if __name__ == '__main__':
    main()
