#Overview 

This project is implementation of Bugs-algorithms for mobile robots in V-REP robot simulator.

You can see demo video [here](https://youtube.com/)

#How to use?

1. If you don't have then you must to install Python and [V-REP simulator](http://www.coppeliarobotics.com/downloads.html).

2. Start V-REP and open some scene from `scenes/` folder, then start simulation.

3.  If you don't use x64 Linux then look at official documentation. It's similar to this tutorial.
 
    To use the remote API functionality in your Python script, you will need following 3 items:

    * vrep.py
    * vrepConst.py
    * remoteApi.dll, remoteApi.dylib or remoteApi.so (depending on your target platform)
    
    Above files are located in V-REP's installation directory, under `programming/remoteApiBindings/python`.
    
    ``Note:`` You can add last version of these files manually!
    
    `remoteApi.so` - to folder with `main.py` script.
    
    `vrep.py` and `vrepConst.py` - to folder `bug/vrep/`.
    
4. run `main.py`.