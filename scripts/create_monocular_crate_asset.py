"""Deterministic wood-tone visual patches, generic feature texture for Gazebo."""
from pathlib import Path
import random
import xml.etree.ElementTree as ET

def main():
    root=Path(__file__).resolve().parents[1]/'models/monocular_textured_box'
    root.mkdir(parents=True,exist_ok=True)
    doc=ET.Element('sdf',{'version':'1.9'});model=ET.SubElement(doc,'model',{'name':'monocular_textured_box'})
    ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',{'name':'body'})
    collision=ET.SubElement(link,'collision',{'name':'collision'})
    ET.SubElement(ET.SubElement(ET.SubElement(collision,'geometry'),'box'),'size').text='2 3 6'
    def visual(name,position,size,color):
        v=ET.SubElement(link,'visual',{'name':name});ET.SubElement(v,'pose').text=' '.join(map(str,(*position,0,0,0)))
        ET.SubElement(ET.SubElement(ET.SubElement(v,'geometry'),'box'),'size').text=' '.join(map(str,size))
        material=ET.SubElement(v,'material')
        for field in ['ambient','diffuse']:ET.SubElement(material,field).text=' '.join(map(str,(*color,1)))
    visual('crate_base',[0,0,0],[2,3,6],[.5,.26,.09])
    rng=random.Random(2718);colors=[(.15,.07,.025),(.35,.17,.06),(.65,.38,.17),(.9,.65,.32)]
    dims=[2.,3.,6.]
    for axis,normal in [(0,-1),(1,-1),(1,1)]:
        other=[i for i in range(3) if i!=axis];counts=[round(dims[i]/.25) for i in other]
        for i in range(counts[0]):
            for j in range(counts[1]):
                position=[0.,0.,0.];size=[.245,.245,.245]
                position[axis]=normal*(dims[axis]/2+.002);size[axis]=.003
                position[other[0]]=-dims[other[0]]/2+(i+.5)*.25
                position[other[1]]=-dims[other[1]]/2+(j+.5)*.25
                visual(f'wood_{axis}_{normal}_{i}_{j}',position,size,colors[rng.randrange(4)])
    ET.ElementTree(doc).write(root/'model.sdf',encoding='utf-8',xml_declaration=True)
    (root/'model.config').write_text('<model><name>Monocular textured crate</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>Generic wood-tone patches; collision remains 2 x 3 x 6 m box.</description></model>')

if __name__=='__main__':main()
