//
// Created by tlab-uav on 24-9-6.
//

#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FSMStateName {
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDDOWN,
    FIXEDSTAND,
    RL,
};

enum class FSMMode {
    NORMAL,
    CHANGE
};

#endif //ENUMCLASS_H
