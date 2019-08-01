#ifndef BOAT_HPP
#define BOAT_HPP

#include<list>
#include<vector>
#include<sstream>
#include<ariadne.hpp>
using namespace Ariadne;

//costruisce l'automa in base al numero di raffiche di vento
std::vector<HybridAutomaton> create_wind_boat(double mass, double inertia, double motor_force, double wave_angle,
                                double wave_speed, double friction, double angular_friction,
                                const std::vector<double>& wind_start_time,
                                const std::vector<double>& wind_end_time,
                                const std::vector<double>& wind_torque) {
 
    // i 3 vettori devono avere la stessa dimensione
    assert(wind_start_time.size() == wind_end_time.size() &&
            wind_end_time.size() == wind_torque.size());
    

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
    RealConstant gamma("gamma", Decimal(angular_friction));
    RealConstant d_coef("d_coef", 0.1_decimal);

    DottedRealAssignments dyn = {
        dot(X) = V*cos(Theta) + d_coef*Vo*cos(Theta_o),
        dot(Y) = V*sin(Theta) + d_coef*Vo*sin(Theta_o),
        dot(Theta) = Omega,
        dot(V) = Fm/m - mu*V*V/m,
        dot(Omega) = -V*V*sin(Theta_r)/I - gamma*Omega*V/I + WindTorque/I
    };

    HybridAutomaton boat("boat");
    boat.new_mode( dyn );


    // automa vento
    RealVariable T("T");

    StringVariable wind_state("wind_state");
    StringConstant start("start");

    DottedRealAssignments wind_dyn = {
        dot(T) = 1.0_decimal
    };

    // genero gli stati e le relative transizioni
    double delta_time = 0.1;
    HybridAutomaton wind("wind");
    wind.new_mode( wind_state|start, { let(WindTorque) = 0.0_decimal }, wind_dyn );
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
        wind.new_mode( wind_state|wind_on, { let(WindTorque) = Decimal(wind_torque[i]) }, wind_dyn );
        wind.new_mode( wind_state|wind_off, { let(WindTorque) = 0.0_decimal }, wind_dyn );

        // genero le transizioni necessarie
        wind.new_transition( wind_state|last, on_transition, wind_state|wind_on
                , { next(T) = T }
                , T>=Decimal(wind_start_time[i] - delta_time), EventKind::PERMISSIVE );
        wind.new_invariant( wind_state|last, T<=Decimal(wind_start_time[i]), must_on_transition);

        wind.new_transition( wind_state|wind_on, off_transition, wind_state|wind_off
                , { next(T) = T }
                , T>=Decimal(wind_end_time[i] - delta_time), EventKind::PERMISSIVE );
        wind.new_invariant( wind_state|wind_on, T<=Decimal(wind_end_time[i]), must_off_transition);

        last = wind_off;
    }

    std::vector<HybridAutomaton> wind_boat({wind, boat});

    return wind_boat;
}

#endif