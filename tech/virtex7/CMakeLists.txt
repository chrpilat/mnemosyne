CONFIGURE_FILE(virtex7_memlib.yaml virtex7_memlib.yaml)

INSTALL(FILES ${CMAKE_BINARY_DIR}/tech/virtex7/virtex7_memlib.yaml DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

INSTALL(FILES WRAP_RAMB16_S1_S1.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES WRAP_RAMB16_S2_S2.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES WRAP_RAMB16_S4_S4.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES WRAP_RAMB16_S9_S9.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES WRAP_RAMB16_S18_S18.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES WRAP_RAMB16_S36_S36.v DESTINATION share/tech/virtex7 PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
