add_plugin(SCDebug)

target_sources(SCDebug PRIVATE
    SCDebug.cpp
    SCDebug.h
)

target_source_group(SCDebug "RenderPasses")
