# Hexacopter MATLAB Simulation Framework

A comprehensive MATLAB framework for simulating, controlling, and analyzing rigid-body hexacopter dynamics with advanced trajectory planning, multiple controller options, and parameter adaptation capabilities.

## 🎯 Project Overview

This framework provides a complete simulation environment for hexacopter research and development, featuring:

- **Advanced Dynamics**: SE(3) rigid-body dynamics with optional ground contact modeling
- **Trajectory Generation**: Multiple predefined trajectories (hover, circle, Lissajous, helix, etc.)
- **Controller Suite**: PD, feedlinearization, feedforward controllers with adaptive capabilities
- **Parameter Adaptation**: Real-time estimation of mass, center of gravity, and inertia properties
- **Visualization**: Live 3D visualization with URDF support or fallback stick model
- **Analysis Tools**: Comprehensive logging, summary plots, and tracking metrics

> **Note**: Despite the repository name, this is a standard hexacopter simulation framework. The tilt-specific logic has been removed, and the model now represents a conventional hexacopter with variable allocation capabilities.

## 🚀 Quick Start

### Prerequisites

- **MATLAB**: Recent version (R2020b or later recommended)
- **Robotics System Toolbox**: Optional, for URDF visualization
- **System Requirements**: Standard MATLAB installation with sufficient memory for 3D visualization

### Installation

1. Clone or download the repository
2. Open MATLAB and navigate to the project root directory
3. Run the startup script to configure the environment

```bash
git clone https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB.git
cd VariableTiltHexacopterMATLAB
```

```matlab
% From MATLAB: change into the repo folder and add paths
cd 'path/to/VariableTiltHexacopterMATLAB'
startup
```

### Running Demos

#### Basic Simulation

```matlab
% Run nominal simulation with predefined trajectory
run_nominal_demo

% Run adaptive simulation with payload drop scenario
run_adaptive_demo
```

#### Custom Configuration

```matlab
startup

% Create configuration object
cfg = vt.config.Config();

% Set trajectory parameters
cfg.setTrajectory('lissajous3d', 1.2);
cfg.setTrajectoryMethod('precomputed');

% Select controller
cfg.setController('Feedforward');

% Configure adaptation
cfg.setAdaptation('none');

% Set simulation parameters
cfg.setSimParams(0.005, 30); % timestep, duration

% Finalize configuration
cfg.done();

% Run simulation
sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save(true, 'summary');
```

## 📊 Features

### Dynamics Engine

- **SE(3) Rigid-Body Dynamics**: Full 6-DOF motion modeling
- **Ground Contact**: Optional ground interaction modeling

### Trajectory Generation

- **Hover**: Stationary position maintenance
- **Circle**: Circular path following
- **Lissajous**: Complex 3D trajectories
- **Helix**: Spiral motion patterns
- **Custom**: User-defined trajectory generation

### Controller Options

- **PD Controller**: Proportional-Derivative control
- **Feedforward Controller**: Model-based feedforward control
- **FeedLin**: Nonlinear feedback linearization
- **Adaptive Controllers**: Real-time parameter estimation

### Visualization

- **URDF Support**: Advanced 3D visualization with Robotics System Toolbox
- **Fallback Model**: Lightweight stick model when URDF unavailable
- **Live Updates**: Real-time state visualization
- **Summary Plots**: Comprehensive post-simulation analysis

## 🛠️ Configuration Guide

The main entry point is `vt.config.Config` (a fluent configuration object). Configure what you need, then call `cfg.done()` to finalize time steps and trajectory timing.

### Basic Configuration

```matlab
startup

cfg = vt.config.Config();

% Trajectory
cfg.setTrajectory('lissajous3d', 1.2);        % name, cycles
cfg.setTrajectoryMethod('precomputed');       % 'precomputed' | 'modelreference'

% Controller
cfg.setController('Feedforward');             % 'PD' | 'Feedforward' | 'FeedLin'
cfg.setPotentialType('liealgebra');           % 'liealgebra' | 'separate'
cfg.setAdaptation('none');                    % 'none' | 'euclidean' | 'geo-aware' | 'geo-enforced' | 'euclidean-boxed'

% Timing
cfg.setSimParams(0.005, 30);                  % sim_dt, duration (s)
cfg.setControlParams(0.005);                  % control_dt (s)

% Visualization
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(100);                   % update every N control steps
cfg.setLiveUrdfEmbedding(false);
cfg.setPlotLayout('column-major');            % 'row-major' | 'column-major'

cfg.done();
```

### Adaptive Payload-Drop Example

```matlab
startup

duration = 30;
cfg = vt.config.Config();

cfg.setTrajectory('lissajous3d', 1.25);
cfg.setTrajectoryMethod('precomputed');

cfg.setController('Feedforward');
cfg.setPotentialType('liealgebra');
cfg.setAdaptation('euclidean');

cfg.setSimParams(0.005, duration);
cfg.setAdaptationParams(0.005);
cfg.setControlParams(0.01);

% payload: mass (kg), CoG (m), drop time (s), startWithTrueValues
cfg.setPayload(1.5, [0.115; 0.05; -0.05], 2*duration/3, false);

cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(500);
cfg.setLiveUrdfEmbedding(true);
cfg.setPlotLayout('column-major');

cfg.done();
```

## 📁 Project Structure

```
VariableTiltHexacopterMATLAB/
├── src/                          # Core framework source code
│   ├── +vt/                      # Main package namespace
│   │   ├── +sim/                 # Simulation runner
│   │   ├── +plant/               # Hexacopter dynamics
│   │   ├── +ctrl/                # Controllers and potentials
│   │   ├── +traj/                # Trajectory generators
│   │   ├── +plot/                # Visualization tools
│   │   └── +metrics/             # Performance metrics
├── run/                          # Demo scripts
├── results/                      # Simulation outputs
├── LICENSE                       # MIT License
├── README.md                     # This file
└── startup.m                     # Environment initialization
```

## 🧪 Simulation Outputs

### Results Directory

Results are saved automatically under `results/`, split by nominal vs adaptive runs:

```
results/
├── nominal/
│   └── yyyymmdd_HHMMSS_<traj>_<ctrl>_<potential>/
│       ├── sim_data.mat
│       ├── summary_nominal.png
│       ├── live_summary.png        # if live summary is enabled
│       └── urdf_view.png           # if URDF view is created
└── adaptive/
    └── yyyymmdd_HHMMSS_<traj>_<ctrl>_<potential>/
        ├── sim_data.mat
        ├── summary_adaptive.png
        ├── live_summary.png        # if live summary is enabled
        └── urdf_view.png           # if URDF view is created
```

`sim_data.mat` contains `logs`, `metrics`, `runInfo`, and `cfg` (plus `est` for adaptive runs).

### Summary Plots

`sim.save(true, 'summary')` generates a single summary figure:

- **3D Path**: desired vs actual
- **XY Path**: top-down view
- **Altitude**: z vs time
- **Position + Orientation**: stacked time-series
- **Linear + Angular Velocity**: stacked time-series
- **Force + Torque (Wrench)**: stacked time-series

For adaptive runs, the summary also includes:

- **Mass + CoG Estimates**
- **Principal Inertia Estimates**
- **Off-Diagonal Inertia Estimates**

`sim.save(true, 'all')` additionally saves standalone and stacked figures (PNG) into the same results folder.

## 🔧 Customization

### Adding New Controllers

Controller selection is currently driven by `cfg.setController(type, ...)` and implemented in the wrench-level controller.

Typical workflow:

1. Add a new controller type string to `src/+vt/+config/Config.m` validation in `setController(...)`
2. Implement the new control law in `src/+vt/+ctrl/WrenchController.m` (add a new `case` for your type)
3. Re-run a demo and save with `sim.save(true, 'summary')`

### Custom Trajectories

Trajectory selection is driven by `cfg.setTrajectory(name, cycles)`.

Typical workflow:

1. Create a new trajectory class under `src/+vt/+traj/` (subclass `vt.traj.TrajectoryBase`)
2. Register it in the trajectory factory (`src/+vt/+traj/TrajectoryFactory.m`)
3. Add the new name to `src/+vt/+config/Config.m` validation in `setTrajectory(...)`

### Visualization Customization

```matlab
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(200);           % update every N control steps
cfg.setLiveUrdfEmbedding(true);       % reserve an axes slot for URDF viewer
cfg.setPlotLayout('row-major');       % 'row-major' | 'column-major'

% Optional (advanced): tweak axis behavior directly
cfg.viz.dynamicAxis = true;
cfg.viz.axisPadding = 2.0;
```

## 🔍 Troubleshooting

### Common Issues

#### MATLAB Path Issues

```matlab
% Ensure all paths are added
startup

% Verify package structure
which vt.sim.SimRunner
```

#### Visualization Problems

- **URDF Not Found**: Fallback to stick model automatically
- **Graphics Performance**: Increase live update interval, disable URDF embedding, or disable live view
- **3D Rendering Issues**: Check MATLAB graphics drivers

#### Simulation Stability

- **Large Time Steps**: Reduce timestep for stiff systems
- **Numerical Issues**: Check initial conditions and parameter values
- **Integration Errors**: Verify ODE solver settings

### Performance Optimization

```matlab
% Optimize for speed
cfg.setSimParams(0.02, 15);           % Larger timestep, shorter duration
cfg.enableLiveView(false);            % Disable live visualization

% Or keep live view but reduce update load
cfg.setLiveUrdfEmbedding(false);
cfg.setLiveUpdateRate(500);
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork** the repository
2. **Create** a feature branch
3. **Document** changes in the README
4. **Submit** a pull request

### Development Setup

```matlab
% Add repo paths
startup

% Run a demo
run_nominal_demo
```

## 📚 References

- **SE(3) Dynamics**: Rigid body motion in 3D space
- **Adaptive Control**: Real-time parameter estimation techniques
- **Trajectory Planning**: Optimal path generation methods

## 📞 Support

For questions and support:

- **Issues**: [GitHub Issues](https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB/issues)
- **Discussions**: [GitHub Discussions](https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB/discussions)
- **Email**: <g202404900@kfupm.edu.sa>, <olanrewajufarooq@yahoo.com>

## 🚀 Version History

### v1.0.0 (Current)

- Initial stable release
- Complete simulation framework
- Multiple controller options
- Advanced trajectory generation
- Parameter adaptation capabilities

### Planned Features

- [ ] Geometric-Aware Adaptive Control
- [ ] Sensor Noise Modeling
- [ ] State Estimation Integration

## 📄 Citations

If you use this framework in your research, please cite one of our papers:

### Paper 1: Indirect Geometric Adaptive Control on SE(3) for a Hexacopter With Online Generalized Inertia and CoG Estimation

```bibtex
@misc{olanrewaju_indirect_geometric_adaptive_control,
  title  = {Indirect Geometric Adaptive Control on SE(3) for a Hexacopter With Online Generalized Inertia and CoG Estimation},
  author = {Farooq Olanrewaju and Aymen Benyahia and Sami El-Ferik and Ramy Rashad},
  note   = {Please replace this entry with the final publication details (venue/year/DOI) once available.}
}
```

### Paper 2: From Modeling to Control Allocation: A Geometric Approach to Variable-Tilt Hexacopter

```bibtex
@misc{olanrewaju_modeling_to_control_allocation_variable_tilt_hexacopter,
  title  = {From Modeling to Control Allocation: A Geometric Approach to Variable-Tilt Hexacopter},
  author = {Farooq Olanrewaju and Ziad Shoeib and Ahmed Abdelrazeq and MD Tarique bin Hamid and Ramy Rashad},
  note   = {Please replace this entry with the final publication details (venue/year/DOI) once available.}
}
```

**Note**: The first paper focuses on adaptive control with online parameter estimation, while the second paper covers the complete modeling and control allocation approach for variable-tilt hexacopters.

---

**Happy Flying!**

*For more information, visit the [project repository](https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB).*
