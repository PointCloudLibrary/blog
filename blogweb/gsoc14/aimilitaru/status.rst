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

		In the previous phase, it was presented how to obtain a statistical model from a set of face-meshes. The next step in our project is to "*match*" the mean face of the database, with the face of a random person, like the one in the picture below:


                .. image:: images/target.png
                        :width: 650px
                        :height: 300px
                        :align: center


		The matching is done by applying alternatively the following methods.



        * **Rigid Registration**

		

		This method is very similar to the Iterative Closest Point Cloud algorithm, because the goal is to estimate a rotation matrix and a translation vector that would move the average face to an optimal position, near the face of the kinect. Basically, it is required to minimize the error :math:`\epsilon =  \sum ||\vec {y} - (R \cdot \vec{x} + \vec{t})||^2` and this is done by calculating the solution of this system in the least square sense. In order to calculate this solution, the system is first linearized using the Jacobian matrix.

		Of course this process is applied iteratively, and below are presented a few stages of positioning of the model over the scan:

                .. image:: images/debug_1.png
                        :width: 650px
                        :height: 300px
                        :align: center


                .. image:: images/debug_2.png
                        :width: 650px
                        :height: 300px
                        :align: center


                .. image:: images/debug_4.png
                        :width: 650px
                        :height: 300px
                        :align: center


                .. image:: images/debug_5.png
                        :width: 650px
                        :height: 300px
                        :align: center





        * **Non-Rigid Registration**

		

		Once the model is roughly aligned, we need to modify the shape of the model to match the face from the scan. For this we make use of the eigenvectors computed in the previous phase and we calculate the optimal solution of this system: :math:`\vec {y} = P \cdot \vec{d} + \vec{model}`, where :math:`P` is the matrix of eigenvectors, :math:`\vec{model}` is the current form of the model and :math:`\vec{d}` is the vector of basis coefficients that need to be determined. 

		However, there is on more constraint to be applied and that is to minimize the sum :math:`\sum_i \frac{d_i}{\sigma_i}`, where :math:`\sigma_i` is the eigenvalue of the corresponding eigenvector. Therefore, to the Jacobian matrix of this system, we need to add a diagonal matrix with :math:`\frac{1}{\sigma_i}` on the diagonal and multiplied by a certain weight. 

		The purpose of this regualrization is to determine to what degree the face should be deformed. The eigenvectors are stored in the :math:`P` matrix in decreasing order according to their eigenvalues and their position in this sorting order determines whether they have a greater or a smaller influence on the shaping of the model. When the model is mostly overlapping with the face in the scan, more information can be drawn about the final figure, hence the weight specified above should be smaller . On the other hand, if the model is not yet aligned with the scan, the deforming should be smaller and thus the weight should be bigger. Below you can see how the model looks for several values of the weight:

                .. image:: images/weight1.png
                        :width: 650px
                        :height: 300px
                        :align: center

                .. image:: images/weight2.png
                        :width: 650px
                        :height: 300px
                        :align: center

		Notice that the shaping process tends to return the same effect if the weight of the regularizing constraint exceeds a certain value.


        * **Results**

		

		As mentioned above, these functions are applied alternatively for a few number of times, and the following results were obtained:

                .. image:: images/result.png
                        :width: 650px
                        :height: 300px
                        :align: center

		The above picture was obtained after one iteration and the following one after 10:

                .. image:: images/result10.png
                        :width: 650px
                        :height: 300px
                        :align: center

		Also, below you can observe the precision of this method, the black figure representing the final version of the model and the green one representing the point cloud of the face:

                .. image:: images/result_overlap.png
                        :width: 650px
                        :height: 300px
                        :align: center

