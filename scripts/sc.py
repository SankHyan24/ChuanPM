from pathlib import WindowsPath, PosixPath
from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    g.create_pass('VBufferSC', 'VBufferSC', {'outputSize': 'Default', 'samplePattern': 'Center', 'sampleCount': 16, 'useAlphaTest': True, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': 'Back', 'useTraceRayInline': False, 'useDOF': True, 'specRoughCutoff': 0.5, 'recursionDepth': 10})
    g.create_pass('ToneMapper', 'ToneMapper', {'outputSize': 'Default', 'useSceneMetadata': True, 'exposureCompensation': 0.0, 'autoExposure': False, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': 'Aces', 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': 'AperturePriority'})
    g.create_pass('RTPM', 'RTPM', {})
    g.add_edge('VBufferSC.vbuffer', 'RTPM.vbuffer')
    g.add_edge('VBufferSC.viewW', 'RTPM.viewW')
    g.add_edge('VBufferSC.throughput', 'RTPM.thp')
    g.add_edge('VBufferSC.emissive', 'RTPM.emissive')
    g.add_edge('RTPM.PhotonImage', 'ToneMapper.src')
    g.mark_output('ToneMapper.dst')
    return g

DefaultRenderGraph = render_graph_DefaultRenderGraph()
try: m.addGraph(DefaultRenderGraph)
except NameError: None
