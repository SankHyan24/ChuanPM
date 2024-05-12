add_plugin(UVMeshPass)

target_sources(UVMeshPass PRIVATE
    UVMeshPass.cpp
    UVMeshPass.h
)

target_source_group(UVMeshPass "RenderPasses")
