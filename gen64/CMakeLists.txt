include_guard(GLOBAL)

add_library(gen64 OBJECT
        $<TARGET_OBJECTS:utils>
        X86mc.h
        definitions.h
        Register.h
        X86mc.cpp
        elf.h
)