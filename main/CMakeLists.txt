idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
		       REQUIRES bmp280 sht21 aht10 request wifi chip_info
		      )

# add_subdirectory(lib/sensor)  # Aggiungi la libreria sensor come sottodirectory
add_subdirectory(components/aht10/include)
register_component()
