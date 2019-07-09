add_subdirectory(nand)
add_subdirectory(nor)
add_library(flash common.c mflash.c)
