add_subdirectory(nand)
add_subdirectory(nor)
add_library(flash common.c FLASHPlugin.c advanced_elf_image.c)
