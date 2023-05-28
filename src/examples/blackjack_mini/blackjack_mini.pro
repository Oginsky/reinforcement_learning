include(../../defs.pri)
include(../../rl/rl.pri)

TEMPLATE = app
DESTDIR = $${bin_dir}
#CONFIG += console c++11
#CONFIG -= app_bundle
#CONFIG -= qt

SOURCES += \
        main.cpp

HEADERS += \
    $$PWD/env.hpp \
    $$PWD/model.hpp \
    agent.hpp \
    linear_agent.hpp

INCLUDEPATH += $${root_dir}
