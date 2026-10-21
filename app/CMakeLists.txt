# Whole-system configuration, composition and output callbacks.
target_sources(sapphire_core PRIVATE pipeline.cpp parameters.cpp)
install(FILES pipeline.hpp parameters.h output.hpp DESTINATION include/app)
