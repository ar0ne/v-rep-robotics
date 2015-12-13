#Overview 

This project is implementation of Bug's algorithms for mobile robots in [V-REP robot simulator](http://coppeliarobotics.com/).

You can see demo video in `/video` folder or on youtube:

* Bug 1

[![Bug 1 algorithm](http://img.youtube.com/vi/rEvZwQH-Fl8/0.jpg)](http://www.youtube.com/watch?v=rEvZwQH-Fl8)

* Bug 2

[![Bug 1 algorithm](http://img.youtube.com/vi/mgBjBiHSAn8/0.jpg)](http://www.youtube.com/watch?v=mgBjBiHSAn8)

* Bug 3

[![Bug 1 algorithm](http://img.youtube.com/vi/tzP2euz-MT4/0.jpg)](http://www.youtube.com/watch?v=tzP2euz-MT4)


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
    
    `vrep.py` and `vrepConst.py` - to folder `src/vrep/`.
    
4. run `$python main.py --help`.

