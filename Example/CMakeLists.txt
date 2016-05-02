
file(GLOB Example_SRC
    "*.cpp"
)

# These are used to create visual studio folders.
source_group(Example FILES ${Example_SRC})

add_executable(Example
	${Example_SRC}
)

target_link_libraries ( Example
	BaluLib
) 