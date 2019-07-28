#include <iostream>
#include <ariadne.hpp>
#include <boat.hpp>
#include <controller_fixed.hpp>
#include <controller_proportional.hpp>
#include <controller_triangle.hpp>

using namespace Ariadne;
typedef GeneralHybridEvolver HybridEvolverType;

struct params {
    double controller_k;
    double controller_heading;
    double mass;
    double inertia;
    double motor_force;
    double wave_angle;
    double wave_speed;
    double friction;
    double initial_velocity;
    double initial_heading;
    std::vector<double> wind_start_time, wind_stop_time, wind_torque;
};

void read_params_from_file(params &p) {
    std::ifstream in_file("configuration.txt");

    assert(!in_file.bad());

    std::string dump;
    int n;
    in_file >> dump >> p.controller_k;
    in_file >> dump >> p.controller_heading;
    in_file >> dump >> p.mass;
    in_file >> dump >> p.inertia;
    in_file >> dump >> p.motor_force;
    in_file >> dump >> p.wave_angle;
    in_file >> dump >> p.wave_speed;
    in_file >> dump >> p.friction;
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

void simulate_wind_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading)
{
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    // RealVariable Xf("Xf"), Yf("Yf");
    RealVariable Omega("Omega");     //linear/angular velocities
    RealVariable Theta_r("Theta_r"); //rudder angle
    RealVariable WindTorque("WindTorque");
    RealVariable T("T");
    TimeVariable time;

    StringVariable position("position");
    StringConstant proportional("proportional");

    StringVariable wind_state("wind_state");
    StringConstant start("start");

    // DiscreteLocation start_loc({position|proportional, wind_state|start});
    DiscreteLocation start_loc({wind_state | start, position | proportional});

    HybridSimulator simulator;
    simulator.set_step_size(0.03125);
    HybridRealPoint initial_point(start_loc,
                                  {
                                      X = 0.0_decimal, Y = 0.0_decimal, Theta = Decimal(initial_heading)
                                      , V = Decimal(initial_V), WindTorque = 0.0_decimal, T = 0.0_decimal, Omega = 0.0_decimal
                                  });
    std::cout << "initial_point=" << initial_point << "\n";

    double sim_time = 140.0;
    HybridTime simulation_time(sim_time, 8);
    std::cout << "simulation_time=" << simulation_time << "\n";

    Orbit<HybridApproximatePoint> trajectory = simulator.orbit(system, initial_point, simulation_time);

    double plot_x = 200, plot_y = 200;

    write("trajectory_simple.txt", trajectory);
    plot("space_trajectory_simple.png", Axes2d(-plot_x <= X <= plot_x, -plot_y <= Y <= plot_y), Colour(0.0, 0.5, 1.0), trajectory);
    plot("V_trajectory_simple.png", Axes2d(0.0 <= time <= sim_time, -20.0 <= V <= 20.0), Colour(0.0, 0.5, 1.0), trajectory);
}

void evolve_wind_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading) {

}

void simulate_simple_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading, DiscreteLocation start_loc)
{
    //instantiate simulation
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    RealVariable Xf("Xf"), Yf("Yf");
    RealVariable Omega("Omega");     //linear/angular velocities
    RealVariable Theta_r("Theta_r"); //rudder angle
    TimeVariable time;

    HybridSimulator simulator;
    simulator.set_step_size(0.03125);
    HybridRealPoint initial_point(start_loc,
                                  {X = 0.0_decimal, Y = 0.0_decimal, Theta = Decimal(initial_heading), V = Decimal(initial_V), Omega = 0.0_decimal});
    double sim_time = 140.0;
    HybridTime simulation_time(sim_time, 8);
    Orbit<HybridApproximatePoint> trajectory = simulator.orbit(system, initial_point, simulation_time);

    double plot_x = 200, plot_y = 200;

    write("trajectory_simple.txt", trajectory);
    plot("space_trajectory_simple.png", Axes2d(-plot_x <= X <= plot_x, -plot_y <= Y <= plot_y), Colour(0.0, 0.5, 1.0), trajectory);
    plot("V_trajectory_simple.png", Axes2d(0.0 <= time <= sim_time, -20.0 <= V <= 20.0), Colour(0.0, 0.5, 1.0), trajectory);
    //plot("rudder_trajectory_simple.png", Axes2d(0.0<=time<=sim_time, -pi/2<=Theta_r<=pi/2), Colour(0.0,0.5,1.0), trajectory);
}

void simulate_simple_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading)
{
    StringVariable position("position");
    StringConstant proportional("proportional");
    StringConstant left("left");
    StringConstant right("right");

    simulate_simple_boat(system, initial_V, initial_heading, position | proportional);
}

void evolve_simple_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading, DiscreteLocation start_loc)
{
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    RealVariable Omega("Omega");     //linear/angular velocities
    RealVariable Theta_r("Theta_r"); //rudder angle
    TimeVariable time;

    double plot_x = 200, plot_y = 200;

    //create evolver
    HybridEvolverType evolver(system);
    //evolver.configuration().set_maximum_enclosure_radius(0.03125);
    evolver.configuration().set_maximum_step_size(2.0);
    evolver.configuration().set_flow_accuracy(0.01); //default 0.00001
    evolver.verbosity = 1;

    // Set the initial set.
    Dyadic Xinit = 0, Yinit = 0, Omega_init = 0;
    HybridSet initial_set(start_loc, {X == Xinit, Y == Yinit, Theta == Decimal(initial_heading), V == Decimal(initial_V), Omega == Omega_init});
    std::cout << "initial_set=" << initial_set << std::endl;

    // Compute the initial set as a validated enclosure.
    double sim_time = 40.0;
    HybridEnclosure initial_enclosure = evolver.enclosure(initial_set);
    std::cout << "initial_enclosure=" << initial_enclosure << "\n\n";

    // Set the maximum evolution time
    HybridTime evolution_time(Decimal(sim_time), 6);
    std::cout << "evolution_time=" << evolution_time << "\n";

    // Compute a validated orbit.
    std::cout << "Computing orbit... \n"
              << std::flush;
    Orbit<HybridEnclosure> orbit = evolver.orbit(initial_set, evolution_time, Semantics::UPPER);
    std::cout << "    done.\n";

    // Write the validated orbit to standard output and plot.
    //std::cout << "Writing orbit... " << std::flush;
    //write("orbit_simple.txt",orbit);
    //std::cout << "done." << std::endl;
    std::cout << "Plotting space orbit... " << std::flush;
    plot("space_orbit_simple.png", Axes2d(-plot_x <= X <= plot_x, -plot_y <= Y <= plot_y), Colour(0.0, 0.5, 1.0), orbit);
    //std::cout << "Plotting velocity orbit... " << std::flush;
    //plot("V_orbit_simple.png", Axes2d(0.0<=time<=sim_time, -20.0<=V<=20.0), Colour(0.0,0.5,1.0), orbit);
    //std::cout << "Plotting rudder orbit... " << std::flush;
    //plot("rudder_orbit_simple.png", Axes2d(0.0<=time<=sim_time, -3.141/2<=Theta_r<=3.141/2), Colour(0.0,0.5,1.0), orbit);
    std::cout << "done.\n\n";
}

void evolve_simple_boat(CompositeHybridAutomaton system, double initial_V, double initial_heading)
{
    DiscreteLocation loc; //for fixed controller
    StringVariable position("position");
    StringConstant proportional("proportional");
    StringConstant left("left");
    StringConstant right("right");
    RealVariable X("X"), Y("Y"), Theta("Theta");
    RealVariable V("V");
    RealVariable Omega("Omega");     //linear/angular velocities
    RealVariable Theta_r("Theta_r"); //rudder angle
    TimeVariable time;

    evolve_simple_boat(system, initial_V, initial_heading, position | proportional);
}

int main()
{
    params p;
    read_params_from_file(p);

    HybridAutomaton prop_controller = create_controller_proportional(p.controller_k, p.controller_heading);

    HybridAutomaton boat = create_wind_boat(p.mass, p.inertia, p.motor_force, p.wave_angle, p.wave_speed, p.friction,
                                                        p.wind_start_time, p.wind_stop_time, p.wind_torque);

    CompositeHybridAutomaton wind_boat({boat, prop_controller});

    std::cout << wind_boat << "\n\n";

    simulate_wind_boat(wind_boat, p.initial_velocity, p.initial_heading);
    evolve_wind_boat(wind_boat, p.initial_velocity, p.initial_heading);

    return 0;
}