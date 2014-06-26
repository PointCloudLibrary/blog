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
   
   
  
  




