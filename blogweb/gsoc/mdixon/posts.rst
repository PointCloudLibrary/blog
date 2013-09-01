.. _mdixon-Including-citations-in-a-blog-post:

.. blogpost::
   :title: Including citations in a blog post
   :author: mdixon
   :date: 6-13-2011

   We recently encountered a problem with the way our blogpost extension was handling citations, but I tracked down the
   problem, and I believe I've got it fixed now.  It should now be possible to cite an article [DixonCVPR2011]_ from 
   within a blogpost, as long as the citation target is also defined in your post.  If you want to see an example, take
   a look at the source for this post :download:`here <posts.rst>`.

   .. [DixonCVPR2011] On analyzing video with very small motions, Michael Dixon et al.
  

.. _mdixon-Bug-fixes:

.. blogpost::
   :title: Bug fixes
   :author: mdixon
   :date: 5-31-2011

   Today I fixed a couple of bugs in our blogpost extension, and I'm writing this post as a test.

   First, :doc:`Alex <../aichim/index>` reported that he was unable to include images inside of a ``.. blogpost::``.
   The problem was solved by adding in a call to ``process_images`` when creating the blogbody content.

     .. image:: pcl_logo.png

   Second, :doc:`Gabe <../goleary/index>` found that the ``:doc:`` role failed to work when included in a post,
   and this turned out to be a similar problem.  The solution was to add a call to ``resolve_references``.

   Finally, :doc:`Pararth <../pararthshah/index>` was unable to link to :download:`downloadable files <posts.rst>`,
   but that appears to be working now, too.

   I suspect I'll need to make a few more fixes along these lines, but I'm afraid that will have to wait until 
   another day.  If any of you run into any more problems, please let me know.

   .. admonition:: *Update*

      Apparently, there's one more problem I'll need to solve to get ``:doc:`` and ``:download:`` working. It looks
      like those commands are assuming our site's root is "pointclouds.org/" instead of "pointclouds.org/gsoc/", so
      all of the automatically generated links are missing the "gsoc/".  I'll have to look into that when I have a 
      chance...

   .. admonition:: *Update 2*

      Fixed. The problem was that a path defined relative to, say, ``pointclouds.org/gsoc/mdixon/`` wouldn't be valid
      in a blogpost that was copied over to ``poinclouds.org/gsoc/index.php``.  The ``resolve_references`` function
      is designed to handle that, but it didn't follow the same conventions as the ``process_images`` or 
      ``process_downloads``, so I needed to pass in a different "docname" in order to get the right behavior.

.. _How-to-install-Sphinx:

.. blogpost:: 
   :title: How to install Sphinx (and some other stuff)
   :author: mdixon
   :date: 5-20-2011

   In my earlier post, I discussed how to write and publish status updates using Sphinx and svn.  However, as you
   might imagine, before you can follow those instructions, you'll need to install Sphinx (and a couple of additional
   dependencies for the extensions we use).  Assuming you don't already have them installed, here's what you need to 
   do:

   First, install `Sphinx 1.0.7 <http://pypi.python.org/pypi/Sphinx>`_.  I have Python 2.6, so this is the one I 
   installed:

   .. code-block:: none

      ~$ easy_install http://pypi.python.org/packages/2.6/S/Sphinx/Sphinx-1.0.7-py2.6.egg
   
   Next, install `python-dateutil <http://labix.org/python-dateutil>`_. (Used by our blog extension to parse the date 
   strings)

   .. code-block:: none

      ~$ easy_install http://labix.org/download/python-dateutil/python-dateutil-1.5.tar.gz

   Finally, install `dvipng <http://www.nongnu.org/dvipng/>`_.  (Used by the pngmath extension to generate LaTeX 
   formulas)

   .. code-block:: none

      ~$ sudo apt-get install dvipng

   After you have these installed, you should be able to "make html" (see earlier instructions) without problems.

   P.S. As you can tell, these instructions assume you're working in Linux. If you're planning to develop in Mac OS X 
   or Windows, then installing dvipng may be a little more complicated for you.  
   (I haven't tried to install it on Windows, but 
   `this page <http://superuser.com/questions/134667/dvipng-with-windows-7>`_ suggests that "The quickest way is to 
   download and install MikTeX - http://miktex.org/2.9/setup ", so you might try that.)
   

.. _How-to-write-a-blog-post:

.. blogpost::
   :title: How to write a blog post
   :author: mdixon
   :date: 5-19-2011

   In order to make sure the mentors and other interested students can keep
   up-to-date about the progress of each project, every student will have their own "developer blog" that they will
   post to every couple of days. In this blog post, I'll describe the infrastructure we'll be using for this and give
   some instructions for how to write your status updates.

   Because we already use `ReStructuredText <http://docutils.sourceforge.net/rst.html>`_ (ReST) as the document format
   for so many other aspects of our site, we've decided to use the same approach for writing and publishing regular 
   status updates.  ReST is a markup language that is designed to be relatively clean and readable. There are also 
   extensions that make it easy to include code snippets and equations in your document, which comes in very handy when
   writing technical content.  If you're unfamiliar with the format, take a look at some of the the source files for 
   our tutorials in ``pcl/trunk/doc/tutorials/content/``; that should give you a few examples of how it looks and what 
   you can do with it. 

   Much like with our documentation and tutorials, the content for the developer blogs will be stored in an svn
   repository.  We're keeping this content separate from our source tree, so to get started, you'll need to check out
   the ``gsocweb`` repository:

   .. code-block:: none

      ~$ svn co svn+ssh://svn@svn.pointclouds.org/gsocweb gsocweb

   If you examine the directory structure, you'll see that the root directory contains a ``source`` directory, which 
   will contain all of the source .rst files for the site, and an ``exts`` directory that contains some useful 
   extensions.  (A ``build`` directory will be created the first time you "make," but these files don't belong in the 
   repository.)

   Under the ``source`` directory, you'll see that every student/developer has their own subdirectory.  Each student 
   directory should have the following files:
   
   * index.rst: A homepage that tells everyone a little bit about you (this page should link to the other pages)
   * roadmap.rst: A page that describes the plans for the project you're working on
   * status.rst: A page that aggregates all of your status updates in one place

   Please take a minute to update the information on your index page. Basic contact information is all that's really
   necessary, but feel free to add whatever additional content you'd like.

   The next step is to write your first status update.  To write a new status update, create a new `.rst` in your 
   directory (you can name it whatever you'd like, but for the sake of discussion, let's call it "blog.rst") and add 
   the following to it:

   .. code-block:: rest

      .. blogpost::
      	 :title: My first status update
	 :author: your_username
	 :date: 5-19-2011

	 Today I learned how to add content to the developer blogs.

	 Here's a code snippet:

	 .. code-block:: c

	    // This is a really boring block of code...
	    int n = 10;
	    for (int i = 0; i < n; ++i)	
	    {
	      printf ("%d\n", i);
	    }

	 And here's an equation:

	 .. math::

	    ax^2 + bx + c = 0
	 
   You can write your posts in any file or subdirectory, and the Sphinx parser will automatically find them and make
   sure they get added to the right blog pages. You aren’t required to keep them all in one file, nor do you need to 
   keep each post in a file of its own. Feel free to organize the files in whatever way makes the most sense to you.

   Just make sure every post is contained within its own ``.. blogpost::`` directive, and don't forget to fill in the 
   correct values for the *author* and *date*. The *title* argument is optional---a default title will be generated if 
   you don’t feel like making one up---but try to use descriptive titles when you can, since that will help your 
   readers to find the posts that are most relevant to them.

   Now that you've created a new file and written a ``.. blogpost`` entry in it, generate the html files like so:

   .. code-block:: none

      ~/gsocweb$ make html

   Preview the new post at ``./build/html/yourusername/blog.html`` directory, and if it looks good, commit your 
   changes. (Remember: since we created a new file for this post, we need to ``svn add`` it before we commit).  

   .. code-block:: none

      ~/gsocweb$ svn add source/your_username/blog.rst
      ~/gsocweb$ svn commit -m "first post"

   The commit will trigger our webserver to rebuild the content, so the update will show up on the website soon after.  
   And that's it!  The post you just wrote will appear in the ``status.html`` file under your directory, and the ten
   most recent posts (by any user) will show up on the main index page at `<http://www.pointclouds.org/gsoc/>`_.

   Thanks for taking the time to update your page. We can't wait to start reading about all the exciting things you'll 
   be developing with us this summer!

   .. note::

      This is still a new set-up, and we might not have all the bugs worked out yet.  If you run into errors or think
      something is amiss, please email the PCL GSoC mailing list (gsoc2011@pointclouds.org).


.. blogpost::
   :title: My first status update
   :author: mdixon
   :date: 5-18-2011

   Today I learned how to add content to the developer blogs.

   Here's a code snippet:

   .. code-block:: c

      // This is a really boring block of code...
      int n = 10;
      for (int i = 0; i < n; ++i)	
      {
        printf ("%d\n", i);
      }

   And here's an equation:

   .. math::

      ax^2 + bx + c = 0
