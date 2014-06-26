William Nguatem 
==========

:email: william.nguatem@googlemail.com	
:project: A Bayesian Approach for fitting Geometric Models in 3D Point Clouds
:mentor: Zoltan-Csaba Marton

About me
--------
I am a graduate student at the Bundeswehr University Munich.

Introduction
-------------
PCL provides a greate frame work based on the ransac family of algorithms for fitting basic geometric primitives e.g Planes, Cylinders, e.t.c.
In this work however, we aim at solving the model fitting tasks using a Bayesian approach. Amongst others, this approach has the advantage of being very flexible and
generic i.e. both parametric and non-parametric (e.g. curves, surfaces using splines) models can be fitted.

Basics of Monte-Carlo Simulation
--------------------------------

Importance Sampling Vs. RANSAC paradigm
----------------------------------------
I'll start this section with a question:

- why should one still be using the RANSAC paradigm for model fitting in these days of modern multi-core computer systems ?

Here, I argue that on a multi-core system, simple Monte-Carlo simulation based alternative (Importance Sampling) combined with a Model Selection 
out performs ransac in most cases. RANSAC may be better if the number of iterations is quite low and the data is quite clean, i.e. within a few iterations, RANSAC can attained the required probability of accepting a sampled model.
In all other cases, I'll believe an IS could be better, at least in terms of speed. Below is a sketch of the algorithm:

.. code-block:: c++
for(int i = 0; i < num_of_samples; i++)
{
                sample[i] = get_sample(); //sample from a proposal distribution
                weight[i] = likelihood_estimation( sample[i] ); //compute the weight of the proposal
}

//weight normalization [optional]
for(int i = 0; i < num_of_samples; i++)
                likelihood[i] = weight[i]/max_weight;

///model selection: get the MAP model
map_sample = get_sample_with_max_weight(weight[i])

For the geometric model fitting problem, I'll take plane fitting as a running example since this is very straight forward. the major goal is to estimate the MAP sample, though one can still get say the MMSE sample estimator using the optional weight normalization stage in the code snippet above. 
On our modern multi-core computer systems, we can run the loop in the above algorithm in parallel since all the samples are i.i.d and there is no coupling from one sample to the other within the loop.




For the geometric model fitting problem, my major goal is to estimate the MAP sample, though I can still get say the MMSE sample estimator using the optional weight normalization stage in the code snippet above. 
On our modern multi-core computer systems, we can run the loop in the above algorithm in parallel since all the samples are i.i.d and there is no coupling from one sample to the other within the loop.

My question is, why should one still be using RANSAC or say Metropolis –Hastings Sampler for model fitting in these days of modern multi-core computer systems ? 
MH-Sampler has the Markovian setting and RANSAC has also a coupling within its main loop. These makes both the MH-Sampler and the RANSAC paradigm not suitable for parallelizing on our modern computers.

I’ve got these doubts while making my proposals using uniform samples generated from Mersenne-Twister algorithm. Here, I assumed that all samples from the different cores on my multi-core system are not correlated.
Meanwhile, if these wasn’t the case, I can still generate all the samples for all the proposals on a single core and run only the likelihood_estimation() in parallel since this is the only step which is highly computational demanding.
In each case, my examples are many magnitudes faster than the MH-Sampler or RANSAC.


Brainstorming
=============
This section contains some of the "Bits and Bytes" from the discussions I've had with my mentor Zoltan

Meeting 1-2
-----------

- Study the existing SampleConsensus and Tracking Module.
- What can be taken from these Modules ?
- The Pros and Cons of inheriting from these modules rather than re-inventing the wheel ? 
- Extending the posible model types for say Torrus/ellipsoids
- B-Spline curve fitting in 3D
- Knot/Control point placement problematic
- Lets try first an MCMC Sampling approach for already existing model types (Planes, Lines,Cylinders ...)
- B-Spline surface fitting

Meeting 3
---------

Recent status updates
---------------------

.. blogbody::
  :author: my_username
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
