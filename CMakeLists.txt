SET(HEADERS
  tree.h
)

SET(SOURCES
  tree.cpp
)

covise_add_module(ABA 3DTree ${EXTRASOURCES} ${SOURCES} ${HEADERS})
