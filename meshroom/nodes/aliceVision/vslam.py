__version__ = "1.0"
import sys
from meshroom.core import desc


class vslam(desc.CommandLineNode):
    commandLine = 'python C:\\source_NaviSuiteDeepVision\\users\\pch\\orb_slam\\demo_orb_slam.py {allParams}'

    inputs = [
        desc.File(
            name='input',
            label='Input',
            description='''Input folder with images.''',
            value='',
            uid=[0],
            ),
        desc.BoolParam(
            name='visual3d',
            label='Visual3d',
            description='''''',
            value='',
            uid=[0],
            ),
    ]

    outputs = [
        desc.File(
            name='output',
            label='Output',
            description='''Output folder.''',
            value=desc.Node.internalFolder,
            uid=[],
            ),
        desc.File(
                name='outputJson',
                label='outputJson',
                description='''Output Json.''',
                value=desc.Node.internalFolder + "data.json",
                uid=[],
                )
    ]
    