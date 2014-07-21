My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: aimilitaru


.. blogpost::
  :title:  Statistical Face Model ( First phase )
  :author: aimilitaru
  :date: 22-06-2014

        * **Introduction**

		The goal of this project is to implement a program that will modify the expressions of several scanned faces according to the facial expressions captured by a RGBD camera.

		The first step is to create a statistical model based on a training database of faces. The training set used so far was the one provided by the FaceWarehouse project and it consisted of 3D meshes stored in **.obj** files. For further information, please consult the following link: http://gaps-zju.org/facewarehouse/


        * **Aproach**

		For each face in the training set, a column vector *S* was created and it contained the coordinates for every vertice of the mesh. Afterwards, the avearage vector and the covariance matrix were calculated. 
		Normally, the covariance matrix would be calculated as :math:`\frac{1}{m} \sum_{i=1}^{m} (S_i - \overline{S}) \cdot (S_i - \overline{S})`, however one should note that this matrix is 34530 by 34530 and in order to compute the statistical model, the most significant eigenvectors are required. To speed up the calculations, a matrix :math:`T` was formed by joining the :math:`(S_i - \overline{S})` vectors and the eigenvectors for :math:`T^t \cdot T` were calculated. It is important to note that the size of :math:`T^t \cdot T` is determined by the number of faces and that the eigenvectors of the covariance matrix can be obtained by left multiplying :math:`T` to the eigenvectors of :math:`T^t \cdot T`.
		Once the eigenvectors are calculated, the statistical model is obtained according to the formula: :math:`S_{model} = \overline{S} + \sum_{i=1}^{m-1} \alpha_i \cdot s_i` , where :math:`\alpha_i` is the weight of an eigenvector, determined by multiplying a random number in the range **[-2,2]** with the corresponding eigenvalue.
		The final results of this phase are presented below. The average face is:



                .. image:: images/snapshot01.png
                        :width: 650px
                        :height: 300px
                        :align: center

		And the model is:


                .. image:: images/snapshot00.png
                        :width: 650px
                        :height: 300px
                        :align: center

		As you can see, the model obtained is a bit flattened compared to the mean face, that is because in the training set the majority of the faces are a bit rounded, however this project needs a model to take into consideration several types of faces, and this is why we need to consider the covariance of the samples in the database.


        * **Feature Steps**

		- For this model, only the vertices of the faces were used, however the texture coordinates also need to be taken into consideration. Unfortunately, the database does not provide any information about the colors as of yet. Once the data is available the model needs to be adapted for this feature
		- Once the statistical model is fully configured, a 3D registration algorithm must be applied to project the facial expression of a testing sample to the model. 





        * **References**

		T\. Vetter and V. Blanz, A Morphable Model For The Synthesis Of 3D Faces, Max-Planck-Institut, Tubingen, Germany


.. blogpost::
  :title:  Rigid and Non-Rigid Transformation ( Second phase )
  :author: aimilitaru
  :date: 17-07-2014

        * **Introduction**

		In the previous phase, it was presented how to obtain a statistical model from a set of face-meshes. The next step in our project is to "*match*" the mean face of the database, with the face obtain from a 3D Camera scan, like the one in the picture below:


                .. image:: images/scan.jpg
                        :width: 650px
                        :height: 300px
                        :align: center


		The matching is done by applying alternatively the following methods.



        * **Rigid Transformation**

		

		This method is very similar to the Iterative Closest Point Cloud algorithm, because the goal is to estimate a rotation matrix and a translation vector that would move the average face to an optimal position, near the face of the kinect. Basically, it is required to minimize the error :math:`\epsilon =  \sum ||\vec {y} - (R \cdot \vec{x} + \vec{t})||^2` and this is done by calculating the Jacobian matrix of this system.

		Of course this process is applied iteratively, and below are presented a few stages of positioning of the model over the scan:

                .. image:: images/initial1.jpg
                        :width: 650px
                        :height: 300px
                        :align: center


                .. image:: images/initial2.jpg
                        :width: 650px
                        :height: 300px
                        :align: center

		The correspondences between the points is determined by comparing the orientation of the vertices from the model with the closest points from the scan. Here you can see an example with the correspondences. Note that at the edges of the scan, the corresponces tend to form pyramids, because a lot of points from the model have the same closest neighbour.


                .. image:: images/correspondences.jpg
                        :width: 650px
                        :height: 400px
                        :align: center




        * **Non-Rigid Transformation**

		

		Once the model is roughly aligned, we need to modify the shape of the model to match the face from the scan. For this we make use of the eigenvectors computed in the previous phase and we calculate a Jacobian matrix for the system: :math:`\vec {y} = P \cdot \vec{d} + \vec{model}`, where :math:`P` is the matrix of eigenvectors, :math:`\vec{model}` is the current form of the model and :math:`\vec{d}` is the vector of basis coefficients that need to be determined. 

		However, there is on more constraint to be applied and that is to minimize the sum :math:`\sum_i \frac{d_i}{\sigma_i}`, where :math:`\sigma_i` is the eigenvalue of the corresponding eigenvector. Therefore, to the Jacobian matrix of this transformation, we need to add a diagonal matrix with :math:`\frac{1}{\sigma_i}` on the diagonal.



        * **Results**

		

		As mentioned above, these functions are applied alternatively for a few number of times, and the following results were obtained:

                .. image:: images/result.jpg
                        :width: 650px
                        :height: 300px
                        :align: center

		The above picture was obtained after one iteration and the following one after 10:

                .. image:: images/result10.jpg
                        :width: 650px
                        :height: 300px
                        :align: center











