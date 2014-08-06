My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: wnguatem


.. blogpost::
  :title: Bayesian Approach for fitting Geometric Models
  :author: wnguatem
  :date: 26-06-2014
  
   Hola! This is my first blog post and I'll start directly with a question:
   
   why should one still be using the RANSAC paradigm in it's vanilla form for geometric model segmentation in these days of modern multi-core computer systems ?
   
   I'll argue that on a multi-core system, simple Monte-Carlo simulation based alternative, Parallel Sampling Consensus (PaSAC) combined with a Model Selection out-performs the vanilla RANSAC in most cases. 
   However though, RANSAC may be better if the data is quite clean, i.e. contains very little outliers. In this case RANSAC can attain the required probability of accepting a sampled model within very few iterations, thus not running all the maximum number of iterations. In all other cases, I believe an PaSAC is far more better, at least in terms of speed. As simple as it is, below is a sketch of this algorithm:
   
   .. code-block:: c++

	for(int i = 0; i < max_iterations; i++)
	{
                sample[i] = get_sample(); //similar to sampling from a proposal distribution
                weight[i] = likelihood_estimation( sample[i] ); //compute the weight of the proposal
	}	

   .. code-block:: c++

	//weight normalization [optional]
	for(int i = 0; i < max_iterations; i++)
                likelihood[i] = weight[i]/max_weight;

   .. code-block:: c++

	//model selection: get the MAP sample
	map_sample = get_sample_with_max_weight(weight[i])

   As always, simple things first and a bit of theory. I'll take plane segmentation as a running example to demonstrate the performance boost you can get on a multi-core system. The above algorithm is making so-called independent and identically distributed (i.i.d.) samples. These samples combined with their weights will give us an approximation of a distribution over planes. This idea of drawing i.i.d. samples to approximate a probability distribution function (pdf) is the core of Monte-Carlo simulation methods and is one of the main thrust in my project. Meanwhile, having the pdf, we can easily make inference. For the geometric model segmentation problem, the major goal is to estimate the maximum a posteriori (MAP) sample. This is just the sample with the highest weight. Also of interest is the  minimum mean square error (MMSE) estimator. This is particularlly important in tracking applications. Using the optional weight normalization stage in the code snippet above. On our modern multi-core computer systems, we can run the loop in the above algorithm in parallel since all the samples are i.i.d. and there is no coupling from one sample to the other within the loop as in the RANSAC case.

   At the moment, I've made a copy of the Sample Consensus Module, renamed this to mcmc module and implemented an mcmc segmentation class for most of the popular model types (Plane, Cylinder, Sphere, Line, ...) already defined in the sample consensus and segmentation modules. I parallelized the PaSAC algorithm using thread pooling concept of boost.asio and boost.thread.
 
   The Figures below show the performance annalysis on three datasets gained from dense image matching and LiDAR. I found it a bit more interessting to test these algorithms on huge data sets containing millions of 3D points. 

                Dataset "Dublin" with over 8 million 3D points from LiDAR.

                .. image:: img/dublincloud.png
                        :width: 1732px
                        :height: 970px
                        :align: center
 
                .. image:: img/dublin.png
                        :width: 1732px
                        :height: 970px
                        :align: center

                Dataset "Facade" from dense image matching and downsampled to 5 million points.

                .. image:: img/facadecloud.png
                        :width: 1732px
                        :height: 970px
                        :align: center

                .. image:: img/facade.png
                        :width: 1732px
                        :height: 970px
                        :align: center

                Dataset "Building" from dense image matching and downsampled to 200000 points.

                .. image:: img/buildingcloud.png
                        :width: 1732px
                        :height: 970px
                        :align: center

                .. image:: img/building.png
                        :width: 1732px
                        :height: 970px
                        :align: center

		The runtime performance analysis of MSAC from pcl vs. PaSAC was made for the plane segmentation example. For this results, a DELL Precision 650, 8xcore, running Windows 7 and VS2010 was used. All time measurements where done using pcl::console::TicToc. An MSAC based likelihood function was used for PaSAC. Notice the effect on increasing the inlier threshold on the runtime.

   		In my next post, I'll discuss my implementation of the three major Markov Chain Monte Carlo (MCMC) algorithms namely Importance Sampling (very similar to PaSAC), Metropolis-Hastings (MH) and the more general reversible jump MCMC (rjMCMC) algorithms. All discussions will be focused on how to use the MCMC algorithms in the sequential evolving data scenario i.e. Tracking (Yes, we are awere of the Tracking library in PCL) and fitting competing models (e.g. fitting curves using splines with an unknown number and locations of knots or control points of splines e.t.c.). Also, since mcmc samples are drawn from the model space rather than from the data space as in RANSAC, it might be challenging to just extends the existing SampleConsensusModel Class.


    	
	
   
  
  




