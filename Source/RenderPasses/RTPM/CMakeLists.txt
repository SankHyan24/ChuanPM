add_plugin(RTPM)

target_sources(RTPM PRIVATE
    RTPM.cpp
    RTPM.h
    PMCollect.cs.slang
    PMGenerate.rt.slang
)

target_copy_shaders(RTPM RenderPasses/RTPM)
target_source_group(RTPM "RenderPasses")
