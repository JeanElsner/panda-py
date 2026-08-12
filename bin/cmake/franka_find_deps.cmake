# Injected into libfranka's build via CMAKE_PROJECT_INCLUDE for libfranka
# >= 0.14.0, which locates its dynamic model through Pinocchio.
#
# Pinocchio's exported config pulls in urdfdom, and urdfdomExport.cmake declares
# console_bridge::console_bridge in its link interface without any find_package
# call bringing that imported target into scope:
#
#   CMake Error at /usr/lib/urdfdom/cmake/urdfdomExport.cmake:55:
#     The link interface of target "urdfdom::urdfdom_model" contains:
#       console_bridge::console_bridge
#     but the target was not found.
#
# libfranka 0.18.0 and later find these themselves, but 0.14.x through 0.17.x do
# not, so define the targets before any of those export files are evaluated.
# QUIET rather than REQUIRED: on versions that already handle this, or that do
# not need it at all, the lookups are simply redundant.
find_package(console_bridge QUIET)
find_package(tinyxml2 QUIET)
find_package(assimp QUIET)
