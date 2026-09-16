#!/usr/bin/env python3
"""Translate yard so its first sharp A* corner is (60,0), extend approach floor."""
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
def build():
    tree=ET.parse(ROOT/'worlds/iris_mppi_industrial_yard.sdf')
    world=tree.getroot().find('world');world.set('name','iris_mppi_yard_runup60')
    for model in world.findall('model'):
        pose=model.find('pose'); values=list(map(float,pose.text.split()))
        values[0]+=45.1; values[1]-=1.9
        if model.get('name')=='asphalt':
            values[:2]=[40,0]
            for size in model.findall('.//geometry/box/size'):size.text='120 60 0.3'
        pose.text=' '.join(map(str,values))
    ET.indent(tree,space='  ')
    tree.write(ROOT/'worlds/iris_mppi_yard_runup60.sdf',encoding='utf-8',xml_declaration=True)
if __name__=='__main__':build()
