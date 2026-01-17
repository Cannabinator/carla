# CARLA V2V Research Platform

### Setting up the workspace

1. Download a pre compiled version of Carla 0.9.16 either for windows or for a Ubuntu workstation. [https://github.com/carla-simulator/carla/releases]  
    
2. setup a virtual environment of your choice using the requirements.txt file
3. start venv and run the `start_server.py` script. You can reach the frontend dashboard on [localhost:8000] 
4. after accessing the frontend dashboard you can set the Server adress of the computer that will run the actual carla server for you when using a windows computer as carla server maybe port 2000 is not reachable from outside by default.
      - to enable port 2000 on windows run the command below in admin powershell.
      - ``` New-NetFirewallRule -DisplayName "Carla Server" -Direction Inbound -Protocol TCP -LocalPort 2000 -Action Allow -Profile Private,Domain ```
5. Start the Carla server `.\CarlaUE4.exe -carla-rpc-port=2000`

    

DONT CHANGE THIS FILE !

### Components
1. **BSM Protocol**