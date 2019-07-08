#ifndef CONTR_PROP_HPP
#define CONTR_PROP_HPP

#include<ariadne.hpp>
using namespace Ariadne;

HybridAutomaton create_controller_proportional(double prop_constant, double heading) {
    RealVariable Theta_r("Theta_r"), Theta("Theta");
    RealConstant Kp("Kp", Decimal(prop_constant)), Theta_c("Theta_c", Decimal(heading));
    StringVariable position("position");
    StringConstant left("left"), right("right"), proportional("proportional");

    DiscreteEvent lockLeft("lockLeft"), unlockLeft("unlockLeft");
    DiscreteEvent lockRight("lockRight"), unlockRight("unlockRight");
    DiscreteEvent mustUnlockRight("mustUnlockRight"), mustUnlockLeft("mustUnlockLeft");


    HybridAutomaton controller("controller");
    controller.new_mode(position|proportional, {let(Theta_r) = Kp*(Theta - Theta_c)});
    controller.new_mode(position|left, {let(Theta_r) = -pi/2});
    controller.new_mode(position|right, {let(Theta_r) = pi/2});

    controller.new_transition(position|proportional, lockLeft, position|left, Theta-Theta_c <= -pi/(2*Kp), EventKind::URGENT);
    controller.new_transition(position|proportional, lockRight, position|right, Theta-Theta_c >= pi/(2*Kp), EventKind::URGENT);
    controller.new_transition(position|left, unlockLeft, position|proportional, Theta-Theta_c >= -pi/(2*Kp)+0.1_decimal, EventKind::URGENT);
    controller.new_transition(position|right, unlockRight, position|proportional, Theta-Theta_c <= pi/(2*Kp)-0.1_decimal, EventKind::URGENT);

    //controller.new_invariant(position|left, Theta-Theta_c <= -pi/(2*Kp)+0.1_decimal, mustUnlockLeft);
    //controller.new_invariant(position|right, Theta-Theta_c >= pi/(2*Kp)-0.1_decimal, mustUnlockRight);

    return controller;
}

#endif