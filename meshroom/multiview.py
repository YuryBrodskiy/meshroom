import os
import fnmatch
import re
import json

from meshroom.core.graph import Graph, GraphModification
# Multiview pipeline version
__version__ = "1.0"


def findFiles(data_source, patterns):
    rules = [re.compile(fnmatch.translate(pattern), re.IGNORECASE) for pattern in patterns]
    outFiles = []
    if data_source.find(".json") == -1:
        folder = data_source
        for name in os.listdir(folder):
            for rule in rules:
                if rule.match(name):
                    filepath = os.path.join(folder, name)
                    outFiles.append(filepath)
                    break
    else:
        json_file = data_source
        with open(json_file) as f:
            data = json.load(f)
            for item in data:
                outFiles.append(item["TexturePath"])
    return outFiles


def photogrammetry(inputFolder='', inputImages=(), inputViewpoints=(), output=''):
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
    graph = Graph('Photogrammetry')
    with GraphModification(graph):
        sfmNodes, mvsNodes = photogrammetryPipeline(graph)
        cameraInit = sfmNodes[0]
        if inputFolder:
            images = findFiles(inputFolder, ['*.jpg', '*.png'])
            cameraInit.viewpoints.extend([{'path': image} for image in images])
        if inputImages:
            cameraInit.viewpoints.extend([{'path': image} for image in inputImages])
        if inputViewpoints:
            cameraInit.viewpoints.extend(inputViewpoints)

    if output:
        texturing = mvsNodes[-1]
        sfm = sfmNodes[-1]
        graph.addNewNode('Publish', output=output, inputFiles=[
            sfm.outputViewsAndPoses,
            texturing.outputMesh,
            texturing.outputXYZ,
            texturing.outputMaterial,
            texturing.outputTextures])
    return graph


def photogrammetryPipeline(graph):
    """
    Instantiate a complete photogrammetry pipeline inside 'graph'.

    Args:
        graph (Graph/UIGraph): the graph in which nodes should be instantiated

    Returns:
        list of Node: the created nodes
    """
    sfmNodes = sfmPipeline(graph)
    mvsNodes = mvsPipeline(graph, sfmNodes[-1])

    # store current pipeline version in graph header
    graph.header.update({'pipelineVersion': __version__})

    return sfmNodes, mvsNodes


def sfmPipeline(graph):
    """
    Instantiate a SfM pipeline inside 'graph'.
    Args:
        graph (Graph/UIGraph): the graph in which nodes should be instantiated

    Returns:
        list of Node: the created nodes
    """
    cameraInit = graph.addNewNode('CameraInit')

    featureExtraction = graph.addNewNode('FeatureExtraction',
                                         input=cameraInit.output)
    imageMatching = graph.addNewNode('ImageMatching',
                                     input=featureExtraction.input,
                                     featuresFolders=[featureExtraction.output])
    featureMatching = graph.addNewNode('FeatureMatching',
                                       input=imageMatching.input,
                                       featuresFolders=imageMatching.featuresFolders,
                                       imagePairsList=imageMatching.output)
    structureFromMotion = graph.addNewNode('StructureFromMotion',
                                           input=featureMatching.input,
                                           featuresFolders=featureMatching.featuresFolders,
                                           matchesFolders=[featureMatching.output])
    return [
        cameraInit,
        featureExtraction,
        imageMatching,
        featureMatching,
        structureFromMotion
    ]


def mvsPipeline(graph, sfm=None):
    """
    Instantiate a MVS pipeline inside 'graph'.

    Args:
        graph (Graph/UIGraph): the graph in which nodes should be instantiated
        sfm (Node, optional): if specified, connect the MVS pipeline to this StructureFromMotion node

    Returns:
        list of Node: the created nodes
    """
    if sfm and not sfm.nodeType == "StructureFromMotion":
        raise ValueError("Invalid node type. Expected StructureFromMotion, got {}.".format(sfm.nodeType))

    prepareDenseScene = graph.addNewNode('PrepareDenseScene',
                                         input=sfm.output if sfm else "")
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
    #
    # meshFiltering = graph.addNewNode('MeshFiltering',
    #                           input=meshing.output)
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
        # meshFiltering,
        texturing
    ]


def sfmAugmentation(graph, sourceSfm, withMVS=False):
    """
    Create a SfM augmentation inside 'graph'.

    Args:
        graph (Graph/UIGraph): the graph in which nodes should be instantiated
        sourceSfm (Node, optional): if specified, connect the MVS pipeline to this StructureFromMotion node
        withMVS (bool): whether to create a MVS pipeline after the augmented SfM branch

    Returns:
        tuple: the created nodes (sfmNodes, mvsNodes)
    """
    cameraInit = graph.addNewNode('CameraInit')

    featureExtraction = graph.addNewNode('FeatureExtraction',
                                         input=cameraInit.output)
    imageMatchingMulti = graph.addNewNode('ImageMatchingMultiSfM',
                                          input=featureExtraction.input,
                                          featuresFolders=[featureExtraction.output]
                                          )
    featureMatching = graph.addNewNode('FeatureMatching',
                                       input=imageMatchingMulti.outputCombinedSfM,
                                       featuresFolders=imageMatchingMulti.featuresFolders,
                                       imagePairsList=imageMatchingMulti.output)
    structureFromMotion = graph.addNewNode('StructureFromMotion',
                                           input=featureMatching.input,
                                           featuresFolders=featureMatching.featuresFolders,
                                           matchesFolders=[featureMatching.output])
    graph.addEdge(sourceSfm.output, imageMatchingMulti.inputB)

    sfmNodes = [
        cameraInit,
        featureMatching,
        imageMatchingMulti,
        featureMatching,
        structureFromMotion
    ]

    mvsNodes = []

    if withMVS:
        mvsNodes = mvsPipeline(graph, structureFromMotion)

    return sfmNodes, mvsNodes
