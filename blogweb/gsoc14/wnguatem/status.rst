My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: wnguatem


.. blogpost::
  :title: Importance Sampling and Model Selection versus RANSAC Paradigm
  :author: wnguatem
  :date: 26-06-2014
  
   Hola! This is my first blog post and I'll start directly with a question:
   
   why should one still be using the RANSAC paradigm for geometric model segmentation in these days of modern multi-core computer systems ?
   
   I'll argue that on a multi-core system, simple Monte-Carlo simulation based alternative (Importance Sampling) combined with a Model Selection out-performs RANSAC in most cases. 
   However though, RANSAC may be better if the number of iterations is quite low and the data is quite clean, i.e. contains very little outliers. Only in this case can
   RANSAC attain the required probability of accepting a sampled model within very few iterations thus not running all the maximum number of iterations. In all other cases, I believe an Important Sampling (IS) 
   is far more better, at least in terms of speed. Below is a sketch of the IS algorithm:
   
   .. code-block:: c++

	for(int i = 0; i < num_of_samples; i++)
	{
                sample[i] = get_sample(); //sample from a proposal distribution
                weight[i] = likelihood_estimation( sample[i] ); //compute the weight of the proposal
	}	

   .. code-block:: c++

	//weight normalization [optional]
	for(int i = 0; i < num_of_samples; i++)
                likelihood[i] = weight[i]/max_weight;

   .. code-block:: c++

	//model selection: get the MAP sample
	map_sample = get_sample_with_max_weight(weight[i])

   As always, simple things first and a bit of theory. I'll take plane segmentation as a running example to demonstrate the performance boost you can get on a multi-core system. The above algorithm is making so-called independent and identically distributed (i.i.d.) samples. These samples combined with their weights will give us an approximation of a distribution over planes. This idea of drawing i.i.d. samples to approximate a probability distribution function (pdf) is the core of Monte-Carlo simulation methods. Meanwhile, having the pdf, we can easily make inference. For the geometric model segmentation problem, the major goal is to estimate the maximum a posteriori (MAP) sample. This is just the sample with the highest weight. Also of interest is the  minimum mean square error (MMSE) estimator using the optional weight normalization stage in the code snippet above. On our modern multi-core computer systems, we can run the loop in the above algorithm in parallel since all the samples are i.i.d. and there is no coupling from one sample to the other within the loop as in the RANSAC case.

   At the moment, I've made a copy of the sample consensus module, renamed this to mcmc module and implemented an mcmc segmentation class for most of the popular model types (Plane, Cylinder, Sphere, Line, ...) already defined in the sample consensus and segmentation modules. In the future, I'll probably create classes that extends	the pcl::SampleConsensus class and implement only the mcmc specific stuff. I parallelized the IS algorithm using thread pooling concept of boost.asio and boost.thread.
 
   The Figure below shows on the top panel results for plane segmentation using IS on a dataset containing 4,000,000pts gained from dense image matching of a boulding scene (green plane), compared to results from using the sample consensus framework, the red segmented plane. In both cases, an MSAC based likelihood function was used.

   .. image:: img/building1.png
	    :width: 1024px
            :height: 768px
            :align: center  
  
   Superimposing both segmented planes still reveals some minor diffences shown within the circle on the bottom panel of the above figure. I believe that there might be an issue with the seeding of the sample consensus module which causes this minor diffences.

   .. image:: img/performanceanalysis1.png
            :align: center 

   I made a performance analysis of the IS combined with MAP estimation compared with the RANSAC framework for segmenting planes on my machine, a DELL Precision 650, 8xcore, running windows 7 and using VS2010. Three different inlier thresholds (0.1m,0.2m and 0.3m) were used. For a given number of maximum RANSAC iterations, three runs of the IS and RANSAC algorithms where made using the three different thresholds mentioned and the average speed in milliseconds was registered (the two middle columns of the table). The results showing the huge speed gain is summarized in the table above. 

   In my next post, I'll discuss two major Markov Chain Monte Carlo (MCMC) algorithms namely Metropolis-Hastings (MH) and the more general reversible jump MCMC (rjMCMC) algorithms. With the MH algorithm, we certainly are not going to be faster than RANSAC but it would be nice to see its advantages and applicability to geometric model segmentation. Therefore, I'll focus on the performance analysis in terms of accuracy compared to RANSAC in an increasing noise scenario for different models, cylinders, cones e.t.c. Whereas, with the rjMCMC, a complete different application scenario will be studied whereby we have different competing models to consider (e.g. fitting curves and surfaces using polynomials of different degrees and different possible locations of knots and/or control points of splines e.t.c.). Also, discussions will be made on how to use the IS algorithms in the sequential evolving data scenario i.e. Tracking (Yes, we are awere of the Tracking library in PCL).


    	
	
   
  
  




