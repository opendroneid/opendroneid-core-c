include_directories(../libopendroneid)

add_executable(odidtest opendroneid_sim.c test_inout.c main.c)
target_link_libraries(odidtest opendroneid m)
