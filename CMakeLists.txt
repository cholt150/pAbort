#--------------------------------------------------------
# The CMakeLists.txt for:                          pAbort
# Author(s):                                   Matt Waltz
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
     wsock32 )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m
      pthread )
endif (${WIN32})


SET(SRC
   main.cpp
)

ADD_EXECUTABLE(pAbort ${SRC})

TARGET_LINK_LIBRARIES(pAbort
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIBS} )
