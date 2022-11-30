# LTR-329ALS

set( srcs "ltr329als.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires hal_esp32 )
set( priv_requires driver hal_esp32 main printfx syslog systiming )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
