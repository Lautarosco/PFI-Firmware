# Embed HTML file for SPIFFS
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES "../spiffs/index.html")

# Add the partition table
idf_build_set_property(PARTITION_TABLE_CUSTOM_FILENAME partitions.csv)
idf_build_set_property(PARTITION_TABLE_CUSTOM_APP_BIN_OFFSET 0x10000)
