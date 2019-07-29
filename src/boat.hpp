#ifndef BOAT_HPP
#define BOAT_HPP

#include<list>
#include<vector>
#include<sstream>
#include<ariadne.hpp>
#include"controller_proportional.hpp"
using namespace Ariadne;

//costruisce l'automa in base al numero di raffiche di vento
HybridAutomaton create_wind_boat(double mass, double inertia, double motor_force, double wave_angle,
                                double wave_speed, double friction,
                                const std::vector<double>& wind_start_time,
                                const std::vector<double>& wind_end_time,
                                const std::vector<double>& wind_torque) {
 
    // i 3 vettori devono avere la stessa dimensione
    assert(wind_start_time.size() == wind_end_time.size() &&
            wind_end_time.size() == wind_torque.size());
    
    RealVariable T("T");

    // HybridAutomaton clock("clock");
    // clock.new_mode( { dot(T) = 1.0_decimal } );

    //automa barca
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V"), Omega("Omega");
    RealVariable Theta_r("Theta_r");
    RealVariable WindTorque("WindTorque");

    RealConstant m("m", Decimal(mass));
    RealConstant I("I", Decimal(inertia));
    RealConstant Fm("Fm", Decimal(motor_force));
    RealConstant Vo("Vo", Decimal(wave_speed)), Theta_o("Theta_o", Decimal(wave_angle));          //angolo/forza onde
    RealConstant mu("mu", Decimal(friction));
    RealConstant d_coef("d_coef", 0.1_decimal);

    StringVariable wind_state("wind_state");
    StringConstant start("start");

    DottedRealAssignments dyn = {
        dot(X) = V*cos(Theta) + d_coef*Vo*cos(Theta_o),
        dot(Y) = V*sin(Theta) + d_coef*Vo*sin(Theta_o),
        dot(Theta) = Omega,
        dot(V) = Fm/m - mu*V*V/m,
        dot(WindTorque) = 0.0_decimal,
        dot(T) = 1.0_decimal,
        dot(Omega) = -V*V*sin(Theta_r)/I - 16.0_decimal*mu*Omega*V/I + WindTorque/I
    };

    // genero gli stati e le relative transizioni
    double delta_time = 0.5;
    HybridAutomaton wind_boat("wind_boat");
    wind_boat.new_mode( wind_state|start, dyn );
    StringConstant last = start;
    for( int i=0; i<wind_torque.size(); i++ ) {
        // genero le stringhe che distinguono gli eventi discreti e le transizioni
        std::stringstream ss;
        ss << i << "_on";
        std::string wind_on_str = ss.str();
        ss.str("");
        ss << i << "_off";
        std::string wind_off_str = ss.str();

        StringConstant wind_on(wind_on_str);
        StringConstant wind_off(wind_off_str);

        ss.str("");
        ss << i << "_on_transition";
        DiscreteEvent on_transition(ss.str());
        ss.str("");
        ss << i << "_off_transition";
        DiscreteEvent off_transition(ss.str());

        ss.str("");
        ss << "must_" << i << "_on_transition";
        DiscreteEvent must_on_transition(ss.str());
        ss.str("");
        ss << "must_" << i << "_off_transition";
        DiscreteEvent must_off_transition(ss.str());

        // genero i due nuovi stati dell'automa
        wind_boat.new_mode( wind_state|wind_on, dyn );
        wind_boat.new_mode( wind_state|wind_off, dyn );

        // genero le transizioni necessarie
        wind_boat.new_transition( wind_state|last, on_transition, wind_state|wind_on
                , {
                    next(X) = X, next(Y) = Y, next(Theta) = Theta
                    , next(V) = V, next(T) = T, next(Omega) = Omega
                    , next(WindTorque) = Decimal(wind_torque[i])
                }
                // , {next(WindTorque) = Decimal(wind_torque[i])}
                , T>=Decimal(wind_start_time[i] - delta_time), EventKind::PERMISSIVE );
        wind_boat.new_invariant( wind_state|last, T<=Decimal(wind_start_time[i]), must_on_transition);

        wind_boat.new_transition( wind_state|wind_on, off_transition, wind_state|wind_off
                , {
                    next(X) = X, next(Y) = Y, next(Theta) = Theta
                    , next(V) = V, next(T) = T, next(Omega) = Omega
                    , next(WindTorque) = 0.0_decimal
                }
                // , {next(WindTorque) = 0.0_decimal}
                , T>=Decimal(wind_end_time[i] - delta_time), EventKind::PERMISSIVE );
        wind_boat.new_invariant( wind_state|wind_on, T<=Decimal(wind_end_time[i]), must_off_transition);

        last = wind_off;
    }

    // CompositeHybridAutomaton comp({clock, wind_boat});

    return wind_boat;
}


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