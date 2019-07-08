#ifndef CONTR_FIXED_HPP
#define CONTR_FIXED_HPP

#include<ariadne.hpp>
using namespace Ariadne;

HybridAutomaton create_controller_fixed(double rudder_angle) {
    RealVariable Theta_r("Theta_r");
    
    HybridAutomaton controller("controller");
    controller.new_mode({let(Theta_r) = Decimal(rudder_angle)});

    return controller;
}

#endif