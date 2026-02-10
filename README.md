# CARLA V2V Research Platform

Production-ready V2V (Vehicle-to-Vehicle) communication and real-time LiDAR visualization for CARLA Simulator 0.9.16.

## Features

- **V2V Communication**: SAE J2735 BSM protocol implementation with 2 Hz update rate
- **LiDAR Visualization**: Real-time 3D point cloud streaming to web browser
- **Stuck Vehicle Detection**: Automatic detection and recovery of immobile traffic vehicles (NEW)
- **Traffic Manager Integration**: Deterministic traffic simulation with configurable behavior
- **Multiple Output Formats**: Console, CSV, V2V message logs, CARLA debug visualization

### Setting up the workspace
I recommend to use the Dockerfile to set everything up
1. Download a pre compiled version of Carla 0.9.16 either for windows or for a Ubuntu workstation. [https://github.com/carla-simulator/carla/releases]  
    
2. setup a virtual environment of your choice using the requirements.txt file
3. start venv and run the `start_server.py` script. You can reach the frontend dashboard on [[localhost:8000](http://localhost:8000/)] 
4. after accessing the frontend dashboard you can set the Server adress of the computer that will run the actual carla server for you when using a windows computer as carla server maybe port 2000 is not reachable from outside by default.
      - to enable port 2000 on windows run the command below in admin powershell.
      - ``` New-NetFirewallRule -DisplayName "Carla Server" -Direction Inbound -Protocol TCP -LocalPort 2000 -Action Allow -Profile Private,Domain ```
5. Start the Carla server `.\CarlaUE4.exe -carla-rpc-port=2000`
6. You can now start the scenario from the frontend Dashboard.
### Set up with Docker
Just use the following commands as intended

`docker-compose build`       # Build Container
`docker-compose up -d`       # Start Container

You are able now to access the frontend Dashboard unter [[localhost:8000](http://localhost:8000/)]

`docker-compose logs -f`     # View logs
`docker-compose down`        # Stop and remove

## Documentation

- [Stuck Vehicle Detection Guide](STUCK_VEHICLE_DETECTION.md) - Configure automatic recovery of immobile traffic vehicles
- [V2V Implementation](V2V_IMPLEMENTATION.md) - Detailed V2V protocol documentation
- [V2V User Guide](V2V_GUIDE.md) - How to use V2V features
- [Docker Setup](DOCKER.md) - Docker deployment guide