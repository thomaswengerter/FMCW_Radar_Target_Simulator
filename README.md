<<<<<<< HEAD
# FMCW Radar Target Simulator

This repository is developed to simulate automotive FMCW radar backscattering signals from a target list. Multiple targets can be placed in a simulated noisy radar environment and the target's reflections are approximated by a combination of individually moving point targets. Random static clutter is added for realistic results. The output is the 3D radar cube with corresponding labels for each szenario which can be utilized for detection algorithm validation or for training of deep learning algorithms.

## Getting Started

Download this repository to your local machine and run MATLAB in the unzipped folder. Make suref you have installed a MATLAB version from 2020 or later. The [Phased Array Toolbox](https://de.mathworks.com/products/phased-array.html) is required for this simulation. Download it from MATLAB's built-in toolbox archive.

The main funtion which is run to initiate the simulation is called *TargetSimulation.m*. Five types of road szenarios/target types are currently implemened: 
* No target (outputs noisy spectrum with static targets)
* Manned bicycle with MATLAB's [backscatterBicyclist](https://de.mathworks.com/help/phased/ref/backscatterbicyclistblock.html)
* Pedestrian with MATLAB's [backscatterPedestrian](https://de.mathworks.com/help/phased/ref/backscatterpedestrian.html)
* Car from class *Car.m*
* Synthetic point targets from function *simulateSignal.m*

Initialize the desired target list here with corresponding positions, velocities and specific properties. If you desire to simulate targets specifically for your own radar device, you can change the radar properties in the class *FMCWradar.m*.

From the target initializations, the simulation generates the representative point targets and performs the radar backscattering procedure to calculate the baseband beat signal seen at the radar receiver.


### Prerequisites

* MATLAB version >= 2020
* MATLAB's phased array toolbox

### Deployment

Let's illustrate the functionality of the programm in a little example. After the repository download the file is unzipped to a local folder.


- *Single Target Szenario*:
    Open the *TargetSimulation.m* file. All available target types are listed at the start of the function. Change the number of desired targets/measurements to create a training dataset. The generator initializes random positions and velocities for each target within the radars field of view. However, specific positions can simply be set for each target type to generate special szenarios of interest. If you want to output a plot of the RDmap, add the index of one or more RX antennas in the variable **plotAntennas**.

- *Multi-Target Szenario*:
    Open the *SimulateTargetList.m* file. All available target types are listed at the first **for** loop. Change the **rand** sampling parameters to set the probability for the individual targets occuring randomly in the measurements. The generator initializes random positions, headings and velocities for each target within the radars field of view. After every measurement, the targets are moved one step along the generated trajectory. To setup special szenarios of interest, the trajectory has to be initialized in the *TrajectoryPlanner*. If you want to output a plot of the RDmap, add the index of one or more RX antennas in the variable **plotAntennas**.

Open *FMCWradar.m* to check the radar settings. All parameters like operating frequency, chirp shape and antenna characteristics are initialized in the properties. If you desire to print information about the added gaussian noise level and SNRs, you can set the variable **printNoiseCharacteristics** to *true*.

The 3D range-doppler-azimuth maps (3D radar cube) and the individual target labels are saved in the folder *SimulationData/...*.
Target labels have the following format:
[Range, velocity, azimuth, RadarVelocity, xPosition, yPosition, width, height, heading]

The coordinate system is always relative to the radar device in (0,0,0), looking along the x-axis. 


## Hints

For the final training data generation, a GPU can be utilized to accelerate the simulation process. To use GPU accelerated processing in MATLAB, change the expressions **for** *target = 1:Pedestrians* ... **end** to **parfor** *target = 1:Pedestrians* ... **end**.




## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Thomas Wengerter** - *Initial work* - [thomaswengerter](https://github.com/thomaswengerter)

See also the list of [contributors](https://github.com/thomaswengerter/FMCW_Radar_Target_Simulator/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* MATLAB's work on the phased array toolbox
* Support from the High Frequency Department at the Technical University of Munich

