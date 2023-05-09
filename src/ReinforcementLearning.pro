include(defs.pri)
include(tasks/tasks.pri)

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += object_parallel_to_source
DESTDIR = bin

SOURCES += \
        main.cpp

HEADERS += \
    detail/iagent.hpp \
    detail/iterable_sequence.hpp \
    detail/traits.hpp \
    rl_algo.hpp \
    coin/task.hpp \
    utility.hpp
