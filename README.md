## Building the workspace
- `clean_build.sh`: deletes the install, build, and log folder, then builds the code
- `build.sh`: builds the code without deleting the above folders
- Note if you get a failure to build zfgen21, try downgrading cmake to 3.22.1
- If you get this error: 'internal compiler error: Segmentation fault', it's possible the compiler is running out of stack memory, so do this to possibly overcome the error: `ulimit -s unlimited`

## Simulation Setup
### Setting up the Local Simulator
1. Install the CLI with 
    ```bash
    python3 -m pip install autoverse-cli
    ```
1. Register the software license with 
    ```bash
    avrs launcher register-license ~/workspace/Maveric-AI-a2rl/assets/asf-code19-license
    ```
1. You can run `avrs launcher license-status` and should see
    ```bash
    Launcher License Status: license registered as asf-code19-license
    download key id obtained
    download key obtained
    ```
1. In your workspace folder, **NOT Maveric folder**, download the latest sim by running
    ```bash
    avrs launcher download-simulator a2rl-humble . --update-existing
    ```
    - If you get an error when trying to download, update your autoverse-cli with `python3 -m pip install --upgrade autoverse-cli`
1. After downloading, update the sensor topic names using this python script. This updates both the default and quality gateway topics
    ```bash
    python3 scripts/update_autoverse_topics.py
    ```
    - **NOTE:** Anytime you update the sim, you will have to rerun this script


### NPC Usage
- To spawn an NPC, use the following command:
    ```bash
    # Spawn an NPC, name is optional
    avrs vehicle-replay spawn random --auto-start --relative-dist 10 --rate 1.0 --name my_ncp0

    # Remove specific NPC
    avrs vehicle-replay despawn --name my_npc0

    # Remove all NPCs
    avrs vehicle-replay despawn --all
    ```
- Spawning an NPC with the above command will start the NPC and have it automatically start moving from 10 m ahead. A rate of 1.0 is roughly a 2:15 lap time. You can change that lap time and speed of the NPC with changing that rate value. As an example, 0.8 will have the vehicle at 80% of that speed, 1.1 will have the vehicle at 110% of that speed, etc.
- The despawn commands above will remove the NPC(s)
- You can also record a lap and replay that as the NPC:
    ```bash
    # Record a lap 
    avrs vehicle-replay start-recording my_recording

    # Stop recording
    avrs vehicle-replay stop-recording

    # Deploy an NPC using that recorded lap
    avrs vehicle-replay spawn my_recording --auto-start --relative-dist 10 --rate 1.0
    ```
- When you record a lap, the file will be saved in `autoverse-linux/Linux/Autoverse/Saved/NPC`. You do **not** need to give the full path, just the name of the recording like above. The sim knows where to find it. The same features apply with the rate, relative distance, etc. when replaying a saved lap 
### Documentation
- Documentation on the simulator can be found [here](https://autonoma.notion.site/AutoVerse-Documentation-de08e4334dd84ed2a938ed2309202be0)
- Example CLI commands can be found [here](https://autonoma.notion.site/AutoVerse-CLI-Examples-13ec91f02d1680f2b1e9eed5f30295ba)

## Running the Software

### Running in Sim
Note: Make sure you run the sim setup steps above prior to running the below commands.

In terminal 1:
```bash
cd Maveric-AI-a2rl
./scripts/setup_vcan.sh # this only has to be run the first time running the sim after starting your computer
```
In terminal 2:
```bash
cd Maveric-AI-a2rl
source install/setup.bash
cd path/to/autoverse-linux/Linux
./Autoverse.sh -ResX=1280 -ResY=720 -WINDOWED
```
In terminal 3:
```bash
cd Maveric-AI-a2rl
source install/setup.bash
ros2 launch a2rl_launch ros2_sim_agent_bridge.launch.py
```
## Open vo in RVIZ:
I order to open the vo simulation in RVIZ, the next steps required:
1. in Team10_ws:
```bash
cd /mnt/c/repos/Team10_ws
rviz2
```
2. once the Rviz is open:
    * Uncheck th grid box
    * Set the fixed frame to *map*
    * Add Marker display:
        * Set the topic to: */visualization_marker*
    * Add Marker Array display:
        * Set the topic to */visualization_marker_array*
3.  in the workspace:
```bash
 source the install/setup file
 ```
4. in Team10_ws:
```bash
colcon build && ros2 run vo vo_simulation_node 
```