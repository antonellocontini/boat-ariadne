#ifndef CONTR_TRI_HPP
#define CONTR_TRI_HPP

#include<ariadne.hpp>
#include<boat.hpp>
using namespace Ariadne;

void evolve_tri_system(double initial_V, double initial_heading) {

}

CompositeHybridAutomaton create_tri_controller_system(double mass, double inertia, double motor_force, double wave_angle,
                                double wave_speed, double friction, double prop_constant) {

    HybridAutomaton boat = create_simple_boat(mass, inertia, motor_force, wave_angle, wave_speed, friction);

    RealVariable Theta_r("Theta_r");
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable Xf("Xf"), Yf("Yf");
    RealConstant Kp("Kp", Decimal(prop_constant));

    StringVariable position("position");
    StringConstant left("left"), right("right"), proportional("proportional");

    DiscreteEvent lockLeft("lockLeft"), unlockLeft("unlockLeft");
    DiscreteEvent lockRight("lockRight"), unlockRight("unlockRight");
    DiscreteEvent mustUnlockRight("mustUnlockRight"), mustUnlockLeft("mustUnlockLeft");

    HybridAutomaton rudder_controller("rudder_controller");
    rudder_controller.new_mode(position|proportional, {let(Theta_r) = Kp*(Theta - atan((Yf-Y)/(Xf-X)))});
    rudder_controller.new_mode(position|left, {let(Theta_r) = -pi/2});
    rudder_controller.new_mode(position|right, {let(Theta_r) = pi/2});

    rudder_controller.new_transition(position|proportional, lockLeft, position|left, Theta-atan((Yf-Y)/(Xf-X)) <= -pi/(2*Kp), EventKind::URGENT);
    rudder_controller.new_transition(position|proportional, lockRight, position|right, Theta-atan((Yf-Y)/(Xf-X)) >= pi/(2*Kp), EventKind::URGENT);
    rudder_controller.new_transition(position|left, unlockLeft, position|proportional, Theta-atan((Yf-Y)/(Xf-X)) >= -pi/(2*Kp)+0.1_decimal, EventKind::URGENT);
    rudder_controller.new_transition(position|right, unlockRight, position|proportional, Theta-atan((Yf-Y)/(Xf-X)) <= pi/(2*Kp)-0.1_decimal, EventKind::URGENT);


    RealVariable Distance("Distance");
    Decimal x1(10.0), y1(-1.0);
    Decimal x2(50.0), y2(-1.0);
    Decimal x3(50.0), y3(31.0);

    StringVariable phase("phase");
    StringConstant start("start"), first("first"), second("second"), third("third");

    DiscreteEvent starting("starting"), first_done("first_done"), second_done("second_done"), third_done("third_done");

    HybridAutomaton pos_controller("pos_controller");
    pos_controller.new_mode(phase|start, {dot(Yf) = 0.0_decimal, dot(Xf) = 0.0_decimal});
    pos_controller.new_mode(phase|first, {dot(Yf) = 0.0_decimal, dot(Xf) = 0.0_decimal});
    pos_controller.new_mode(phase|second, {dot(Yf) = 0.0_decimal, dot(Xf) = 0.0_decimal});
    pos_controller.new_mode(phase|third, {dot(Yf) = 0.0_decimal, dot(Xf) = 0.0_decimal});

    pos_controller.new_transition(phase|start, starting, phase|first, {next(Xf) = x1, next(Yf) = y1});
    pos_controller.new_transition(phase|first, first_done, phase|second, {next(Xf) = x2, next(Yf) = y2}, (Xf-X)*(Xf-X) + (Yf-Y)*(Yf-Y) < 2.0_decimal, EventKind::URGENT);
    pos_controller.new_transition(phase|second, second_done, phase|third, {next(Xf) = x3, next(Yf) = y3}, (Xf-X)*(Xf-X) + (Yf-Y)*(Yf-Y) < 2.0_decimal, EventKind::URGENT);
    pos_controller.new_transition(phase|third, third_done, phase|first, {next(Xf) = x1, next(Yf) = y1}, (Xf-X)*(Xf-X) + (Yf-Y)*(Yf-Y) < 2.0_decimal, EventKind::URGENT);

    CompositeHybridAutomaton system({boat,pos_controller,rudder_controller});

    return system;
}

#endif