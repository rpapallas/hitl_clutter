set( BOOST_ROOT "/local/boost_1_64_0" CACHE PATH "Boost library path" )
set( Boost_NO_SYSTEM_PATHS on CACHE BOOL "Do not search system for Boost" )
find_package(Boost)
