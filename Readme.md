ORB2BoWDb
=========

This tool allows one to create a yml database of bag of word features extracted from ORB descriptors captured in a video sequence.

Installation 
------------

First a couple of dependencies: 

Make sure you have catkin, openCV, DLib and DBoW2 installed on your machine. 

For  DLib and DBoW2 you can get them at: 
- DLib: https://github.com/dorian3d/DLib
- DBoW2: https://github.com/ldecamp/DBoW2

For now you'll need my fork of DBoW2 to make it run since I had some missing templates for the ORBDatabase and vocabulary.

To install these two folks: 

```
cd ~ && mkdir workspace && cd workspace
git clone https://github.com/dorian3d/DLib
cd DLib && mkdir build && cd build && cmake .. && sudo make install
cd ~/workspace
git clone https://github.com/ldecamp/DBoW2
cd DBoW2 && mkdir build && cd build && cmake .. && sudo make install
```

Once you've got all the dependencies setup, run the following from the root folder of a catkin workspace 

```
cd src
git clone https://github.com/ldecamp/ORB2BoWDb.git
cd .. && catkin_make
```

This should be installed now.

Execution
---------

There is a couple of knobs one can tweak as described below. 
To make things easy there is an example of launcher in the Data folder. 

You can run it using: 
```
roslaunch orb2bowdb THE_SETTINGS.launch
```

Make sure you've setup the correct camera topic first.

Parameters available: 
1. the ros topic where the images are published. 
2. the path of the output yml file
3. the interval between 2 frames processed by the node. 


The node will keep running until you press the 's' key. 
At this point it will create the database and kill the node. 