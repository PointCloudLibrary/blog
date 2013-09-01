My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Jinwoo Lee

.. blogpost::
	:title: Setting up environment
	:author: Jinwoo Lee
	:date: 5-16-2012

	I prefer Windows and visual studio for development. I also prepared Ubuntu and development tools (cmake, code, build, subversion tools).

 	I'm reading papers and text about particle filter.

	* Min-An Chao; Chun-Yuan Chu; Chih-Hao Chao; An-Yeu Wu, **"Efficient parallelized particle filter design on CUDA,"** Signal Processing Systems (SiPS), 2010 IEEE Workshop on Digital Object Identifier, pp.299-304
	
	TODOs

	* Analysing particle filter and CUDA API on PCL

	* Writing draft headers for particle filter

.. blogpost::
	:title: Studying particle filter on PCL
	:author: Jinwoo Lee
	:date: 5-30-2012

	Particle filter(PF) on PCL is little bit different with general algorithm.
	
	PF on PCL it contains 
		* resample - update each particles
		* weight - calc each particle weights based on coherence between reference cloud and query cloud
		* update - Update representative state of particles
		
	In general PF, motion is given from system with uncertainty, but PF on PCL use camera motion calculated in prev frame.

.. blogpost::
	:title: Design CUDA functions
	:author: Jinwoo Lee
	:date: 6-14-2012

	Design CUDA functions based on PF on PCL and Kinfu.
		
	* ParticleXYZRPY, PointXYZRGB -> should be changed
	* Octree for search nearest point.
	* Random number generator for sampling.
	* Reduction step for 'update' function
	
	* After complete one period, I need to think about more general PF. 
