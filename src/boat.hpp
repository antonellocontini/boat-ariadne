#ifndef BOAT_HPP
#define BOAT_HPP

#include<ariadne.hpp>
using namespace Ariadne;


HybridAutomaton create_simple_boat(double mass, double inertia, double motor_force, double wave_angle,
                                double wave_speed, double friction) {
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V"), Omega("Omega");
    RealVariable Theta_r("Theta_r");

    RealConstant m("m", Decimal(mass));
    RealConstant I("I", Decimal(inertia));
    RealConstant Fm("Fm", Decimal(motor_force));
    RealConstant Vo("Vo", Decimal(wave_speed)), Theta_o("Theta_o", Decimal(wave_angle));          //angolo/forza onde
    RealConstant mu("mu", Decimal(friction));
    RealConstant d_coef("d_coef", 0.1_decimal);

    DottedRealAssignments dyn = {
        dot(X) = V*cos(Theta) + d_coef*Vo*cos(Theta_o),
        dot(Y) = V*sin(Theta) + d_coef*Vo*sin(Theta_o),
        dot(Theta) = Omega,
        dot(V) = Fm/m - mu*V*V/m,
        dot(Omega) = -V*V*sin(Theta_r)/I - 16.0_decimal*mu*Omega*V/I
    };

    HybridAutomaton simple_boat("simple_boat");
    simple_boat.new_mode(dyn);

    return simple_boat;
}

HybridAutomaton create_boat(double mass, double inertia, double motor_force, double wave_angle,
                                double wave_force, double friction) {

    RealVariable X("X"), Y("Y"), Theta("Theta");        //posizione e angolo barca
    RealVariable Vx("Vx"), Vy("Vy"), Omega("Omega");    //velocità lineari/angolari
    RealVariable Theta_r("Theta_r");                    //angolo timone
    RealVariable V_magnitude("V_magnitude"), Friction("Friction");

    RealConstant m("m", Decimal(mass));
    RealConstant I("I", Decimal(inertia));
    RealConstant Fm("Fm", Decimal(motor_force));
    RealConstant Fo("Fo", Decimal(wave_force)), Theta_o("Theta_o", Decimal(wave_angle));          //forza/angolo onde
    RealConstant mu("mu", Decimal(friction));

    HybridAutomaton boat("boat");

    DottedRealAssignments boat_dynamic = {
        dot(X) = Vx,
        dot(Y) = Vy,
        dot(Theta) = Omega,
        //dot(Vx) = Fm/m*cos(Theta) + Fo/m*cos(Theta_o) - mu*(Vx*Vx + Vy*Vy)*cos(Theta),
        dot(Vx) = Fm/m*cos(Theta) + Fo/m*cos(Theta_o) - mu*sqrt(pow(Vx,2) + pow(Vy,2))*Vx/m,  //attrito sull'angolo della velocità(!= da theta)
        //dot(Vx) = Fm/m*cos(Theta) + Fo/m*cos(Theta_o) - mu*V_magnitude*Vx,
        //dot(Vx) = Fm/m*cos(Theta) + Fo/m*cos(Theta_o) - 1000*Vx,

        //dot(Vy) = Fm/m*sin(Theta) + Fo/m*sin(Theta_o) - mu*(Vx*Vx + Vy*Vy)*sin(Theta),
        dot(Vy) = Fm/m*sin(Theta) + Fo/m*sin(Theta_o) - mu*sqrt(pow(Vx,2) + pow(Vy,2))*Vy/m,
        //dot(Vy) = Fm/m*sin(Theta) + Fo/m*sin(Theta_o) - mu*V_magnitude*Vy,
        //dot(Vy) = Fm/m*sin(Theta) + Fo/m*sin(Theta_o) - 1000*Vy,

        dot(Omega) = -(Vx*Vx + Vy*Vy)*sin(Theta_r)/I
    };

    boat.new_mode(boat_dynamic);

    return boat;
}

#endif