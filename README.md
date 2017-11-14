Base a particle simulator using 6 different emitters.

The following keys enable different emitters:
"1": Fire
"2": Water
"3": Magic
"4": Single Bouncy Ball
"5": Snow
"6": Benchmark Fire Test

Use WASD to move around the scene, and the mouse to rotate the camera view.


You can also use "Q" and "E" to rotate the camera horizontally, "O" and "P" to rotate vertically, and "Z" and "X" to move the camera position up and down on the Y Axis. These alternative movements are not constrained, and create some perspective errors. Use with caution.



## Compiling (Instructions from CSCI5607)

You only need to run cmake once. After that, only a "make" is required.

By default, dependencies (GLFW and maybe GLEW) will be cloned into a "lib/" directory.

If you are using XCode or MS Vis. Studio, see [cs5607parsing](https://github.umn.edu/over0219/cs5607parsing) for more details on compiling with CMake. Otherwise, you can compile with the following commands:

	mkdir build
	cd build
	cmake ..
	make -j
	./HW1_App
	
On the linux csdev machines there is an issue with gcc, so remember to run the following commands first:

	module rm soft/gcc/4.5.2
	module initrm soft/gcc/4.5.2
