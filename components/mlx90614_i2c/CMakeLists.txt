set(SOURCES src/mlx90614_i2c/mlx90614_i2c.cpp)
            
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS include
                    REQUIRES CPPI2C)