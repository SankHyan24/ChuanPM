add_plugin(VBufferSC)

target_sources(VBufferSC PRIVATE
    VBufferSC.cpp
    VBufferSC.h
    VBufferSC.rt.slang
    VBufferSC.cs.slang
    VBufferSC.slang
)

target_copy_shaders(VBufferSC RenderPasses/VBufferSC)

target_source_group(VBufferSC "RenderPasses")
