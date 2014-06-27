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
   
   why should one still be using the RANSAC paradigm for model fitting in these days of modern multi-core computer systems ?
   
   I'll argue that on a multi-core system, simple Monte-Carlo simulation based alternative (Importance Sampling) combined with a Model Selection outperforms RANSAC in most cases. 
   However though, RANSAC may be better if the number of iterations is quite low and the data is quite clean, i.e. data contains little outliers and within a few iterations, 
   RANSAC can attained the required probability of accepting a sampled model thus not running all the maximum number of iterations. In all other cases, I believe an Important Sampling (IS) 
   is better, at least in terms of speed. Below is a sketch of the IS algorithm:
   
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
	//model selection: get the MAP model
	map_sample = get_sample_with_max_weight(weight[i])
	
	As always, simple things first, I'll take plane fitting as a running example since this is very straight forward. 
	The above algorithm is making so-called independent and identically distributed (i.i.d.) samples. These samples combined with their weights will 
	give us an approximation of a distribution over planes. This idea of drawing i.i.d. samples to approximate a probability distribution (pdf) is the core of Monte-Carlo simulation methods. 
	Meanwhile, having the pdf, we can easily make inference. For the geometric model fitting problem, the major goal is to estimate the maximum a posteriori (MAP) sample. This is just the sample with the heighest weight.
	Also of interest is the  minimum mean square error (MMSE) estimator using the optional weight normalization stage in the code snippet above. 

	On our modern multi-core computer systems, we can run the loop in the above algorithm in parallel since all the samples are i.i.d. and there is no coupling from one sample to the other within the loop as found in RANSAC.
	
	At the moment, I've made a copy of the sample_consensus modul and renamed this to mcmc modul and implemented an mcmc_segmentation class for the segmentation module. This is very analogous to the sample_consensus module. 
	In the future I'll probably create classes that extends	sample_consensus and implement only the mcmc specific stuff. The currently implemented IS algorithm is parallelized using thread pooling concept of boost.asio and boost.thread. 
	Results from plane fitting on a dataset collected from image matching is shown in the green segmented plane compared to results from
	from the sac_segmentation, the red segmented plane. In both cases, an MSAC based likelihood function was used. The table shows the runtime performance gain of the IS algorithm on my machine (DELL Precision 650) 
	running windows 7 compared to the plane segmentation using the sample consensus framework.
	
	   .. image:: img/building1.png
            :width: 1400px
            :height: 1100px
            :align: center     

	I have also implemented the Metropolis-Hastings (MH) and Gibbs Samplers for geometric model fitting. These algorithms have a markovian property thus cannot make any use of multi-core systems.
    
    Currently, I am implementing the most generic of all MCMC based algorithms, the reversible jump mcmc, (rjmcmc). From now on, I'll introduce some necessary formulars governing Bayesian inference using mcmc.
    The bad thing in rjmcmc and other mcmc samplers is that, stats folks always turn to explain these algorithms such that only other pure stats folks can understand. The good thing is, explain only the practicality behind the scene the algorithms.
	My objective of using rjmcmc is to be able to solve the knot or control point placement problematic in fitting spline curves and surfaces in 3D. 
	
	Agenda for the next meeting:
	Discuss methods of evaluating the correctness/accuracy of these algorithms compared to the already existing sample_consensus MSAC algorithm for different already existing models.
	Discuss how to use the mcmc fitted models within a tracking scenario, i.e. data is evolving over time
	
	
    	
	
   
  
  




