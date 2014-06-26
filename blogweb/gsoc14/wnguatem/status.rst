My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: my_username


.. blogpost::
  :title: Importance Sampling and Model Selection Vs. Ransac Paradigm
  :author: wnguatem
  :date: 26-06-2014
  
  Importance Sampling vs. RANSAC paradigm
----------------------------------------
I'll start this section with a question:

- why should one still be using the RANSAC paradigm for model fitting in these days of modern multi-core computer systems ?

Here, I argue that on a multi-core system, simple Monte-Carlo simulation based alternative (Importance Sampling) combined with a Model Selection 
out performs ransac in most cases. RANSAC may be better if the number of iterations is quite low and the data is quite clean, i.e. within a few iterations, RANSAC can attained the required probability of accepting a sampled model.
In all other cases, I'll believe an IS could be better, at least in terms of speed. Below is a sketch of the algorithm:



