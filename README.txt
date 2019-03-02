Preliminary experiment procedures: (please correct if you notice anything wrong)

1. run physical_runner.py, it will prompt you to enter fitness, ignore it for now and leave it running in the terminal ->
2. save genome.txt and fitness.txt into the SD card ->
3. set up the cylinders ->
4. plug the SD card into Arduino mega ->
5. upload Treebot/Treebot.ino ->
6. open serial monitor, where information about the current trial should be displayed. Fitness will be displayed at the end of the trial, you may also see fitness for the current trial in fitness.txt ->
7. unplug the SD card from Arduino, and plug it into the machine on which you are running physical_runner.py -> 
8. enter the fitness of this run into physical_runner, which should prompt you for another fitness value, leave it for now and continue ->
9. new genome files should have been generated in the project folder, one of which should correspond to the genome and generation asked by physical_runner.py, save this genome into the SD card and rename it genome.txt (overwriting the previous genome.txt) ->
10. go back to step 3 and repeat from there, notice that fitness.txt will be overwritten with new fitnesses after each trial.