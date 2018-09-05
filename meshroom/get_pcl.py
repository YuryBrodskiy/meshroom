# Multiview pipeline version
__version__ = "1.0"

import os
import fnmatch
import re
import json
from meshroom.core.graph import Graph, GraphModification

def getPCLGraph(inputJson='',  output=''):
    """
    Create a new Graph with a complete photogrammetry pipeline.

    Args:
        inputFolder (str, optional): folder containing image files
        inputImages (list of str, optional): list of image file paths
        inputViewpoints (list of Viewpoint, optional): list of Viewpoints
        output (str, optional): the path to export reconstructed model to

    Returns:
        Graph: the created graph
    """
    graph = Graph('GetPCL')
    with GraphModification(graph):
        mvsNodes = mvsPipeline(graph, inputJson)
        # store current pipeline version in graph header
        graph.header.update({'pipelineVersion': __version__})
        
    if output:
        texturing = mvsNodes[-1]
        graph.addNewNode('Publish', output=output, inputFiles=[
                                                               texturing.outputMesh,
                                                               texturing.outputXYZ,
                                                               texturing.outputMaterial,
                                                               texturing.outputTextures])
    return graph

from meshroom.core import desc
def mvsPipeline(graph, inputJson):
    """
    Instantiate a MVS pipeline inside 'graph'.

    Args:
        graph (Graph/UIGraph): the graph in which nodes should be instantiated
        sfm (Node, optional): if specified, connect the MVS pipeline to this StructureFromMotion node

    Returns:
        list of Node: the created nodes
    """

    prepareDenseScene = graph.addNewNode('PrepareDenseScene',
                                         input=inputJson)

    cameraConnection = graph.addNewNode('CameraConnection',
                                        ini=prepareDenseScene.ini)
    depthMap = graph.addNewNode('DepthMap',
                                ini=cameraConnection.ini)
    depthMapFilter = graph.addNewNode('DepthMapFilter',
                                      depthMapFolder=depthMap.output,
                                      ini=depthMap.ini)
    meshing = graph.addNewNode('Meshing',
                               depthMapFolder=depthMapFilter.depthMapFolder,
                               depthMapFilterFolder=depthMapFilter.output,
                               ini=depthMapFilter.ini)
    texturing = graph.addNewNode('Texturing',
                                 ini=meshing.ini,
                                 inputDenseReconstruction=meshing.outputDenseReconstruction,
                                 inputMesh=meshing.output)

    return [
        prepareDenseScene,
        cameraConnection,
        depthMap,
        depthMapFilter,
        meshing,
        texturing
    ]

