uint8 SOURCE_TOPOLOGY=1
uint8 SOURCE_REGION=2
uint8 SOURCE_GRADIENT=3
uint8 TYPE_ENDPOINT=1
uint8 TYPE_JUNCTION=2
uint8 TYPE_LOOP_ANCHOR=3
uint8 TYPE_REGION_MEDOID=10
uint8 TYPE_GRADIENT=20

uint32 id
geometry_msgs/Point position
uint8 source
uint8 type
uint32 degree
float32 traversability
