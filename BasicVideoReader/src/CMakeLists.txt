
set(OFFLINEEXEC offline)

set(ONLINEEXEC online)

set(CLASSES easyhandsegmentation.cpp
            easyhandtracker.cpp
            easyhanddetector.cpp
            contourbasedfingerdetector.cpp
            trackdrawer.cpp
            palmcenterdetector.cpp
            dynamictimewarping.cpp
            skeleton.cpp
            morphology.cpp
            kalmanhandtracker.cpp)

set(OFFLINE main.cpp)

set(ONLINE onlineTrackingMain.cpp)

set(HEADERS ../include/segmentHandEasy.h
            ../include/handsegmentation.h
            ../include/easyhandsegmentation.h
            ../include/easyhanddetector.h
            ../include/fastqueue.hpp
            ../include/handtracker.h
            ../include/easyhandtracker.h
            ../include/rect3d.hpp
            ../include/handdetector.h
            ../include/contourbasedfingerdetector.h
            ../include/trackdrawer.h
            ../include/palmcenterdetector.h
            ../include/dynamictimewarping.h
            ../include/skeleton.h
            ../include/morphology.h
            ../include/kalmanhandtracker.h)

set(ILIBNAME libddconverter)

add_library(${ILIBNAME} STATIC IMPORTED)

set_property(TARGET ${ILIBNAME} PROPERTY IMPORTED_LOCATION ${ILIBPATH}/lib/${ILIBNAME}.a)

add_executable(${OFFLINEEXEC} ${CLASSES} ${OFFLINE} ${HEADERS})

add_executable(${ONLINEEXEC} ${CLASSES} ${ONLINE} ${HEADERS})

target_link_libraries(${OFFLINEEXEC} ${OpenCV_LIBS} ${ILIBNAME})

target_link_libraries(${ONLINEEXEC} ${OpenCV_LIBS})

install(TARGETS ${OFFLINEEXEC} DESTINATION .)

install(TARGETS ${ONLINEEXEC} DESTINATION .)

