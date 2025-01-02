# NASA Lunar Autonomy Challenge Submission

## Project Overview

This repository contains our submission for the NASA Lunar Autonomy Challenge. Our project develops an advanced autonomous navigation system for lunar rovers, designed to map unknown terrain on the lunar surface.

Our approach combines computer vision techniques using OpenCV, reinforcement learning with TensorFlow, and high-fidelity simulation using CARLA. This solution enables a simulated rover to accurately map the surface while navigating the terrain, avoiding hazardous obstacles, and managing limited resources.

## Key Features

- Computer vision-based terrain analysis and obstacle detection using OpenCV
- Reinforcement learning model for adaptive decision-making and path planning
- Real-time image processing for enhanced situational awareness
- Deep learning-powered energy optimization strategies
- High-fidelity simulation using CARLA for realistic environment modeling
- Seamless integration with NASA's lunar surface simulation environment

## Technical Stack

- **OpenCV**: For image processing and computer vision tasks
- **TensorFlow**: For implementing and training our reinforcement learning model
- **CARLA Simulator**: For high-fidelity environment simulation and sensor data generation
- **Python**: Primary programming language
- **NASA Lunar Surface Simulator**: For final testing and validation of our approach

## Setup Instructions

To run and test our submission, follow the steps below:

1. **Download the Lunar Autonomy Challenge Package**
   - Download the package from [https://lac-content.s3.us-west-2.amazonaws.com/LunarAutonomyChallenge.zip](https://lac-content.s3.us-west-2.amazonaws.com/LunarAutonomyChallenge.zip)
   - Extract the downloaded package to a directory of your choice

2. **Read the Documentation**
   - Navigate to `LunarAutonomyChallenge/docs/`
   - Open and carefully read the "Lunar Autonomy Challenge documentation.pdf" file
   - This document contains detailed instructions for setting up the simulator and running the challenge

3. **Set Up the Simulator**
   - Follow the instructions in the documentation PDF to set up the simulator environment
   - This may include installing dependencies, configuring the CARLA simulator, and setting up any required virtual environments

4. **Clone Our Repository**
   ```bash
   git clone https://github.com/alex-tanton/LunarAutonomyChallenge.git
   cd LunarAutonomyChallenge
   ```

5. **Import Our Agent**
   - Locate the `my_agent.py` file in the `/agents` directory of our repository
   - Copy `my_agent.py` to the `/agents` directory in the Lunar Autonomy Challenge package you extracted in step 1

6. **Run the Simulation**
   - Follow the instructions in the documentation PDF to run the simulation
   - Make sure to specify our agent (`my_agent.py`) in `/RunLeaderboard.sh` when running the simulation

7. **View Results**
   - Refer to the documentation PDF for information on where to find and how to interpret the simulation results and performance metrics

## Performance Metrics

Our system's performance in the simulated lunar environment:

**Coming Later**

## Contributors

| Name | Role | GitHub |
|------|------|--------|
| Srikar Desemsetti | Test & Simulation Lead | [@SrikarD123](https://github.com/SrikarD123) |
| Sarayu Kondaveeti | Project Lead | [@sara-k03](https://github.com/sara-k03) |
| Armaan Raina | Development Lead | [@Armaan-Raina](https://github.com/Armaan-Raina) |
| Alex Tanton | Algorithms Lead | [@alex-tanton](https://github.com/alex-tanton) |

## License

This project is licensed under the Apache 2.0 License - see [here](http://www.apache.org/licenses/) for more details.

## Acknowledgments

We thank NASA for organizing this challenge and providing the simulation environment. We also acknowledge the contributions of our team members, advisors, and the open-source community, particularly the OpenCV, TensorFlow, and CARLA teams for their excellent tools and documentation.
