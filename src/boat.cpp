#include <iostream>
#include <ariadne.hpp>
#include <boat.hpp>
#include <controller_fixed.hpp>
#include <controller_proportional.hpp>

using namespace Ariadne;
typedef GeneralHybridEvolver HybridEvolverType;

// struttura contenente i vari parametri 
struct params {
    double sim_time;
    double step_size;
    double flow_accuracy;
    double floor_radius;
    double controller_k;
    double controller_heading;
    double mass;
    double inertia;
    double motor_force;
    double wave_angle;
    double wave_speed;
    double friction;
    double angular_friction;
    double initial_velocity;
    double initial_heading;
    std::vector<double> wind_start_time, wind_stop_time, wind_torque;
};

// legge i parametri dal file configuration.txt, deve essere situato nella directory corrente
void read_params_from_file(params &p) {
    std::ifstream in_file("configuration.txt");

    assert(!in_file.bad());

    std::string dump;
    int n;
    in_file >> dump >> p.sim_time;
    in_file >> dump >> p.step_size;
    in_file >> dump >> p.flow_accuracy;
    in_file >> dump >> p.floor_radius;
    in_file >> dump >> p.controller_k;
    in_file >> dump >> p.controller_heading;
    in_file >> dump >> p.mass;
    in_file >> dump >> p.inertia;
    in_file >> dump >> p.motor_force;
    in_file >> dump >> p.wave_angle;
    in_file >> dump >> p.wave_speed;
    in_file >> dump >> p.friction;
    in_file >> dump >> p.angular_friction;
    in_file >> dump >> p.initial_velocity;
    in_file >> dump >> p.initial_heading;
    in_file >> dump >> n;
    p.wind_start_time.clear();
    p.wind_stop_time.clear();
    p.wind_torque.clear();
    for( int i=0; i<n; i++ ) {
        double d;
        in_file >> d;
        p.wind_start_time.push_back(d);
        in_file >> d;
        p.wind_stop_time.push_back(d);
        in_file >> d;
        p.wind_torque.push_back(d);
    }
}

template <class T>
void write(const char *filename, const T &t)
{
    std::ofstream ofs(filename);
    ofs << t;
    ofs.close();
}

void simulate_wind_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading, double sim_time = 140.0, double floor_size = 200.0)
{
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    RealVariable Omega("Omega");     //linear/angular velocities
    RealVariable Theta_r("Theta_r"); //rudder angle
    RealVariable T("T");
    TimeVariable time;

    StringVariable position("position");
    StringConstant proportional("proportional");

    StringVariable wind_state("wind_state");
    StringConstant start("start");

    DiscreteLocation start_loc({wind_state | start, position | proportional});

    HybridSimulator simulator;
    simulator.set_step_size(0.03125);
    HybridRealPoint initial_point(start_loc,
                                  {
                                      X = 0.0_decimal, Y = 0.0_decimal, Theta = Decimal(initial_heading)
                                      , V = Decimal(initial_V), T = 0.0_decimal, Omega = 0.0_decimal
                                  });
    std::cout << "initial_point=" << initial_point << "\n";

    HybridTime simulation_time(sim_time, 10);
    std::cout << "simulation_time=" << simulation_time << "\n";

    Orbit<HybridApproximatePoint> trajectory = simulator.orbit(system, initial_point, simulation_time);

    double plot_x = floor_size, plot_y = floor_size;

    write("trajectory_wind.txt", trajectory);
    plot("space_trajectory_wind.png", Axes2d(-plot_x <= X <= plot_x, -plot_y <= Y <= plot_y), Colour(0.0, 0.5, 1.0), trajectory);
    plot("angle_trajectory_wind.png", Axes2d(0.0 <= time <= sim_time, -pi <= Theta <= pi), Colour(0.0, 1.0, 0.5), trajectory);
}

void evolve_wind_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading,
            double sim_time = 60.0, double step_size = 8.0, double flow_accuracy = 8.0, double floor_size = 200.0) {
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    RealVariable Omega("Omega");
    RealVariable Theta_r("Theta_r");
    RealVariable T("T");
    TimeVariable time;

    StringVariable position("position");
    StringConstant proportional("proportional");

    StringVariable wind_state("wind_state");
    StringConstant start("start");

    DiscreteLocation start_loc({ wind_state|start, position|proportional});

    HybridEvolverType evolver(system);
    evolver.configuration().set_maximum_step_size(step_size);
    evolver.configuration().set_flow_accuracy(flow_accuracy); //default 0.00001
    evolver.verbosity = 1;

    HybridSet initial_set(start_loc, 
                    { X == 0.0_decimal, Y == 0.0_decimal, Theta == Decimal(initial_heading),
                      V == Decimal(initial_V), Omega == 0.0_decimal, T == 0.0_decimal });
    std::cout << "initial_set=" << initial_set << "\n";

    HybridEnclosure initial_enclosure = evolver.enclosure(initial_set);
    std::cout << "initial_enclosure=" << initial_enclosure << "\n\n";

    HybridTime evolution_time(Decimal(sim_time), 10);
    std::cout << "evolution_time=" << evolution_time << "\n";

    std::cout << "Computing orbit...\n" << std::flush;
    Orbit<HybridEnclosure> orbit = evolver.orbit(initial_set, evolution_time, Semantics::LOWER);
    std::cout << "done.\n";

    double plot_x = floor_size, plot_y = floor_size;
    std::cout << "Plotting space orbit... " << std::flush;
    plot("space_orbit_wind.png", Axes2d(-plot_x <= X <= plot_x, -plot_y <= Y <= plot_y), Colour(0.0, 0.5, 1.0), orbit);
    std::cout << "done.\n";

    std::cout << "Plotting angle orbit... " << std::flush;
    plot("angle_orbit_wind.png", Axes2d(0.0 <= time <= sim_time, -pi <= Theta <= pi), Colour(0.0, 1.0, 0.5), orbit);
    std::cout << "done.\n";

    // NON E' POSSIBILE PLOTTARE VARIABILI DEFINITE SOLO TRAMITE LET???
    // std::cout << "Plotting rudder orbit... " << std::flush;
    // plot("rudder_orbit_wind.png", Axes2d(0.0 <= time <= sim_time, -pi/2 <= Theta_r <= pi/2), Colour(1.0, 0.5, 0.0), orbit);
    // std::cout << "done.\n";
}


int main()
{
    params p;
    read_params_from_file(p);

    HybridAutomaton prop_controller = create_controller_proportional(p.controller_k, p.controller_heading);

    std::vector<HybridAutomaton> sys_vec = create_wind_boat(p.mass, p.inertia, p.motor_force, p.wave_angle, p.wave_speed, p.friction,
                                                        p.angular_friction, p.wind_start_time, p.wind_stop_time, p.wind_torque);
    sys_vec.push_back(prop_controller);

    CompositeHybridAutomaton wind_boat(sys_vec);

    std::cout << wind_boat << "\n\n";

    simulate_wind_boat(wind_boat, p.initial_velocity, p.initial_heading, p.sim_time, p.floor_radius);
    evolve_wind_boat(wind_boat, p.initial_velocity, p.initial_heading, p.sim_time, p.step_size, p.flow_accuracy, p.floor_radius);

    return 0;
}