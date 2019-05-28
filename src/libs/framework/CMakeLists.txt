# core stuff
add_subdirectory(rpicore)
add_subdirectory(rpiworld)

# communication stuff
add_subdirectory(ethercat)
add_subdirectory(canopen)
add_subdirectory(ethernet)
add_subdirectory(socketcan)
add_subdirectory(rtsocketcan)

# generic interface stuff
add_subdirectory(robotarm)
add_subdirectory(robotbase)
add_subdirectory(armkinematics)
add_subdirectory(cartesianposition)
add_subdirectory(io)

