include_directories(../libopendroneid ../libmav2odid ../mavlink_c_library_v2)

add_executable(odidtest opendroneid_sim.c test_inout.c test_mav2odid.c main.c)
target_link_libraries(odidtest opendroneid mav2odid m)
