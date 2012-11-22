cocos_sph
=========

Cocos2d SPH particle fluid dynamics with box2d interaction

Port of ElectroDruid's C++ SPH implementation found on box2d forums.
Link: http://www.box2d.org/forum/viewtopic.php?f=3&t=574&start=70

iPhone 4 can run this 60fps with 500 particles. 

This demo is not working that well as the original version. I couldn't get the verlet version stable enough, so it's turned off. 
Euler integration have problems in interactions with box2d dynamic bodies. 