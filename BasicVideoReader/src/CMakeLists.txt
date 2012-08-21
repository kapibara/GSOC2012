
set(OFFLINEEXEC offline)

set(ONLINEEXEC online)

set(DEMOEXEC demo)

set(CLASSES easyhandsegmentation.cpp
            easyhandtracker.cpp
            easyhanddetector.cpp
            contourbasedfingerdetector.cpp
            trackdrawer.cpp
            palmcenterdetector.cpp
            dynamictimewarping.cpp
            skeleton.cpp
            morphology.cpp
            kalmanhandtracker.cpp
            grabbableball.cpp)

set(OFFLINE main.cpp)

set(ONLINE onlineTrackingMain.cpp)

set(DEMO demo.cpp)

set(HEADERS ../include/handsegmentation.h
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
            ../include/kalmanhandtracker.h
            ../include/grabbableobject.h
            ../include/grabbableball.h)

set(ILIBNAME libddconverter)

add_library(${ILIBNAME} STATIC IMPORTED)

set_property(TARGET ${ILIBNAME} PROPERTY IMPORTED_LOCATION ${ILIBPATH}/lib/${ILIBNAME}.a)

add_executable(${OFFLINEEXEC} ${CLASSES} ${OFFLINE} ${HEADERS})

add_executable(${ONLINEEXEC} ${CLASSES} ${ONLINE} ${HEADERS})

add_executable(${DEMOEXEC} ${CLASSES} ${DEMO} ${HEADERS})

target_link_libraries(${OFFLINEEXEC} ${OpenCV_LIBS} ${ILIBNAME})

target_link_libraries(${ONLINEEXEC} ${OpenCV_LIBS})

target_link_libraries(${DEMOEXEC} ${OpenCV_LIBS})

install(TARGETS ${OFFLINEEXEC} DESTINATION .)

install(TARGETS ${ONLINEEXEC} DESTINATION .)

install(TARGETS ${DEMOEXEC} DESTINATION .)
