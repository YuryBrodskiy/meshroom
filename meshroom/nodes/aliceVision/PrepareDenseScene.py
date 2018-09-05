__version__ = "1.0"

from meshroom.core import desc


class PrepareDenseScene(desc.CommandLineNode):
    commandLine = 'aliceVision_prepareDenseScene {allParams}'
    #size = desc.DynamicNodeSize('input')
    size = desc.NodeSizeFromJson('input')
    inputs = [
        desc.File(
            name='input',
            label='Input',
            description='''SfMData file.''',
            value='',
            uid=[0],
        ),
        desc.ChoiceParam(
            name='verboseLevel',
            label='Verbose Level',
            description='''verbosity level (fatal, error, warning, info, debug, trace).''',
            value='info',
            values=['fatal', 'error', 'warning', 'info', 'debug', 'trace'],
            exclusive=True,
            uid=[],
        ),
    ]

    outputs = [
        desc.File(
            name='ini',
            label='MVS Configuration file',
            description='',
            value=desc.Node.internalFolder + 'mvs.ini',
            uid=[],
            group='',  # not a command line arg
        ),

        desc.File(
            name='output',
            label='Output',
            description='''Output folder.''',
            value=desc.Node.internalFolder,
            uid=[],
        )
    ]

    #def buildCommandLine(self, chunk):
    #    print(type(chunk))
    #    raise TypeError("ups")
    #    return super(PrepareDenseScene,self).buildCommandLine(chunk)