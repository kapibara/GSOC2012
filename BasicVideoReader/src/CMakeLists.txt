
set(OFFLINEEXEC offline)

set(ONLINEEXEC online)

set(CLASSES easyhandsegmentation.cpp
            easyhandtracker.cpp
            handdetector.cpp
   )

set(OFFLINE main.cpp)

set(ONLINE onlineTrackingMain.cpp)

set(HEADERS ../include/segmentHandEasy.h
            ../include/handsegmentation.h
            ../include/easyhandsegmentation.h
            ../include/fastqueue.hpp
            ../include/extractContour.h
            ../include/handtracker.h
            ../include/easyhandtracker.h
            ../include/rect3d.hpp
            ../include/handdetector.h)

set(ILIBNAME libddconverter)

add_library(${ILIBNAME} STATIC IMPORTED)

set_property(TARGET ${ILIBNAME} PROPERTY IMPORTED_LOCATION ${ILIBPATH}/lib/${ILIBNAME}.a)

add_executable(${OFFLINEEXEC} ${CLASSES} ${OFFLINE} ${HEADERS})

add_executable(${ONLINEEXEC} ${CLASSES} ${ONLINE} ${HEADERS})

target_link_libraries(${OFFLINEEXEC} ${OpenCV_LIBS} ${ILIBNAME})

target_link_libraries(${ONLINEEXEC} ${OpenCV_LIBS} ${ILIBNAME})

install(TARGETS ${OFFLINEEXEC} DESTINATION .)

install(TARGETS ${ONLINEEXEC} DESTINATION .)

