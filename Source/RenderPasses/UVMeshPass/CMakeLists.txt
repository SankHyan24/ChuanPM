add_plugin(UVMeshPass)

target_sources(UVMeshPass PRIVATE
    UVMeshPass.cpp
    UVMeshPass.h
    UVMeshPass.3d.slang
)
target_copy_shaders(UVMeshPass RenderPasses/UVMeshPass)

target_source_group(UVMeshPass "RenderPasses")
