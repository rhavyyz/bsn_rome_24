### Team: everyone from Ceará

### The adaptation consists basicly on keeping track on the avarage heart rate of the last minute and verify which state whould be more compatible with this number. If this state is not selected right now the adaptation goes on to switch the current state. The adaptation is trigered when the current avarage heart rate is in a zone of mid or high risk.

### Build the SA-BSN

To compile the SA-BSN

```
catkin_make
```

### Execute the SA-BSN

The SA-BSN's execution relies on a single command, but first you need to ensure roscore to be running.

```
roscore
```

Then, in another terminal.

```
mon launch bsn.launch
```

Finally you need to execute the script that input the real data to SA-BSN

```
cd RoME_execution
python3 script.py
```
