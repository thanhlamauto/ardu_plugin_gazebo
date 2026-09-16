#!/usr/bin/env python3
"""Rebuild the authored industrial yard; no downloaded assets required.

All structural solids share visual/collision geometry. Decorative ribs, doors
and road markings are visual-only, contained within the structural footprint.
This is a synthetic industrial scene, not a surveyed real-world map.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build():
    tree = ET.parse(ROOT / 'worlds/iris_mppi_slalom.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'iris_mppi_industrial_yard')
    for model in list(world.findall('model')):
        world.remove(model)

    def box(name, xyz, size, color, solid=True):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, (*xyz, 0, 0, 0)))
        link = ET.SubElement(model, 'link', name='body')
        for kind in (['collision', 'visual'] if solid else ['visual']):
            element = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(element, 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for field in ('ambient', 'diffuse'):
                    ET.SubElement(material, field).text = color

    box('asphalt', (15, 0, -.15), (65, 50, .3), '.19 .20 .22 1')
    # Buildings bound a service lane; staggered stacked containers force turns.
    box('north_warehouse', (15, 15, 4), (36, 8, 8), '.64 .67 .70 1')
    box('south_warehouse', (15, -15, 4), (36, 8, 8), '.66 .62 .55 1')
    for x in (2, 10, 18, 26):
        box(f'north_loading_door_{x}', (x, 10.995, 1.8), (3.5, .01, 3.6), '.25 .29 .32 1', False)
        box(f'south_loading_door_{x}', (x, -10.995, 1.8), (3.5, .01, 3.6), '.25 .29 .32 1', False)
    for i, (x, y, color) in enumerate(((9, -2, '.65 .18 .12 1'), (21, 2, '.10 .32 .58 1'))):
        # Three 2.6m containers stacked: 7.8m high, route altitude is 5m.
        for level in range(3):
            z = 1.3 + 2.6 * level
            box(f'container_{i}_{level}', (x, y, z), (6, 2.44, 2.6), color)
            for rib in range(14):
                for side in (-1, 1):
                    box(f'rib_{i}_{level}_{rib}_{side}',
                        (x - 2.8 + rib * .43, y + side * 1.218, z),
                        (.06, .008, 2.5), '.75 .75 .72 1', False)
    for x in range(-4, 36, 3):
        box(f'road_dash_{x}', (x, 0, .006), (1.4, .12, .01), '.9 .8 .25 1', False)
    box('loading_pallets', (4, -8, .6), (3, 2, 1.2), '.48 .32 .15 1')
    box('parked_trailer', (28, 8, 1.9), (7, 2.5, 3.8), '.8 .8 .77 1')
    ET.indent(tree, space='  ')
    destination = ROOT / 'worlds/iris_mppi_industrial_yard.sdf'
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    print(destination)


if __name__ == '__main__':
    build()
