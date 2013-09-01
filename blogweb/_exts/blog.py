# The Setup Function
def setup(app):
    app.add_node(blogbody)
    app.add_node(blogpost,
                 html=(visit_blogpost_node, depart_blogpost_node),
                 latex=(visit_blogpost_node, depart_blogpost_node),
                 text=(visit_blogpost_node, depart_blogpost_node))

    app.add_directive('blogbody', BlogbodyDirective)
    app.add_directive('blogpost', BlogpostDirective)
    app.connect('doctree-resolved', process_blogpost_nodes)
    app.connect('env-purge-doc', purge_blogposts)



# The Node Classes
from docutils import nodes

class blogpost(nodes.General, nodes.Element):
    pass

class blogbody(nodes.General, nodes.Element):
    pass

def visit_blogpost_node(self, node):
    self.visit_section(node)

def depart_blogpost_node(self, node):
    self.depart_section(node)



# The Directive Classes
from sphinx.util.compat import Directive
from docutils.parsers.rst import directives

class BlogbodyDirective(Directive):
    required_arguments = 0
    optional_arguments = 0
    option_spec = {'project': directives.unchanged_required,
                   'author': directives.unchanged_required,
                   'nr_days': directives.positive_int,
                   'nr_posts': directives.positive_int,
                   'order': lambda arg: directives.choice(arg, ('ascending', 'descending'))}
    has_content = False

    def run(self):
        env = self.state.document.settings.env
        if not 'project' in self.options:
            self.options['project'] = env.docname.split('/',1)[0];

        node = blogbody('')
        node['options'] = self.options

        return [node]


from sphinx.locale import _
import dateutil.parser
import datetime

class BlogpostDirective(Directive):

    required_arguments = 0
    optional_arguments = 0
    option_spec = {'title': directives.unchanged_required,
                   'date': directives.unchanged_required,
                   'author': directives.unchanged_required,
                   'project': directives.unchanged_required}

    has_content = True

    def run(self):
        env = self.state.document.settings.env

        # Parse options
        if not 'date' in self.options:
            raise NamedError("No date given")

        if not 'author' in self.options:
            raise NamedError("No author given")

        if not 'title' in self.options:
            self.options['title'] = 'Untitled - ' + self.options['date']

        try:
            timestamp = dateutil.parser.parse(self.options['date'])
            self.options['timestamp'] = timestamp
        except:
            raise NamedError('Could not parse date string: %s' % self.options['date'])

        if not 'project' in self.options:
            self.options['project'] = env.docname.split('/',1)[0]
        project = self.options['project']

        # Create section node to contain the blogpost
        targetid = "blogpost-%d" % env.new_serialno('blogpost.container')
        blogpost_node = nodes.section('\n'.join(self.content), ids=[targetid], classes=['blogpost', project])

        # Add title, date, and author nodes
        header_node = nodes.container('',classes=['blogpost-header', project])

        title_text = self.options['title']
        textnodes, messages = self.state.inline_text(title_text, self.lineno)
        title_node = nodes.container('',classes=['blogpost-title', project])
        title_node += textnodes
        title_node += messages
        header_node += title_node

        date_text = self.options['timestamp'].strftime('%A, %B %d, %Y')
        date_node = nodes.container('',classes=['blogpost-date', project])
        date_node += nodes.Text(date_text, date_text)
        header_node += date_node

        # Create an author node that links to back to the author page
        author_node = nodes.container('', classes=['blogpost-author', project])
        author_para = nodes.paragraph('','');
        author_ref = addnodes.pending_xref('rawtext', reftype='doc', refcaption=False, \
                              modname=env.currmodule, classname=env.currclass)
        author_ref['reftarget'] = 'index'
        author_ref['refdoc'] = '/'.join(env.docname.split('/',2)[0:2]) + '/index'
        author_ref['refexplicit'] = True
        author_ref.append(nodes.Text(self.options['author'], self.options['author']))
        author_para += author_ref
        author_node += author_para
        header_node += author_node


        # Create a project node that links to back to the project page
        depth = len(env.docname.split('/')) - 1
        image_filename = '../'*depth + '_static/' + self.options['project'] + '.png'        
        project_node = nodes.container('', classes=['blogpost-project', project])
        project_para = nodes.paragraph('','');
        project_ref = nodes.reference(refuri='') #we'll fill in the refuri later when filling the 'blogbody's
        project_ref['project'] = self.options['project']
        project_ref += nodes.image(uri=image_filename)
        project_para += project_ref
        project_node += project_para
        header_node += project_node

        blogpost_node += header_node

        # Add additional classes, if specified
        if 'class' in self.options:
            classes += self.options['class']

        # Nested parse
        self.state.nested_parse(self.content, self.content_offset, blogpost_node)

        # Store the blogpost info in a list for later
        if not hasattr(env, 'blog_all_blogposts'):
            env.blog_all_blogposts = []

        env.blog_all_blogposts.append({
                'docname': env.docname,
                'lineno': self.lineno,
                'blogpost': blogpost_node.deepcopy(),
                'targetid': targetid,
                'options': self.options
                })

        return [blogpost_node]




# The Event Handlers

def purge_blogposts(app, env, docname):
    if not hasattr(env, 'blog_all_blogposts'):
        return
    env.blog_all_blogposts = [blogpost for blogpost in env.blog_all_blogposts
                              if blogpost['docname'] != docname]


from sphinx import addnodes
from sphinx.util import docname_join
def process_blogpost_nodes(app, doctree, fromdocname):
    # Replace all blogbody nodes with a list of the collected blogposts.
    # Augment each blogpost with a backlink to the original location.

    env = app.builder.env

    for node in doctree.traverse(blogbody):

        # Build a list of blogposts to display
        blogposts_to_display = []
        for blogpost_info in env.blog_all_blogposts:
            author_match = \
                not 'author' in node['options'] or \
                blogpost_info['options']['author'].lower() == node['options']['author'].lower()
            project_match = \
                node['options']['project'].lower() == 'all' or \
                node['options']['project'].lower() == blogpost_info['options']['project'].lower()
            if author_match and project_match:
                blogposts_to_display.append((blogpost_info['options']['timestamp'], blogpost_info))
            else:
                pass

        if not blogposts_to_display:
            node.replace_self([])
            continue

        # Establish the maximum number of posts to display
        if 'nr_posts' in node['options']:
            max_posts = node['options']['nr_posts']
        else:
            max_posts = float("inf")

        # Establish the oldest post to display
        if 'nr_days' in node['options']:
            most_recent_post_date = max(blogposts_to_display, key=lambda x: x[0])[0]
            oldest_post_to_display = most_recent_post_date - datetime.timedelta(days=node['options']['nr_days'])
        else:
            oldest_post_to_display = min(blogposts_to_display, key=lambda x: x[0])[0]

         # Add the blogposts to the document
        blogbody_node = nodes.container('', classes=['blogbody'])
        content = []

        for i_post, (blogpost_date, blogpost_info) in enumerate(sorted(blogposts_to_display, reverse=True)):

            # Terminate early if the maximum number of posts or days is exceeded
            if i_post >= max_posts or blogpost_date < oldest_post_to_display:
                break

            blogpost_node = blogpost_info['blogpost'].deepcopy()

            # Insert into the blogbody
            if 'order' in node['options'] and node['options']['order'].lower() == 'descending':
                for i, x in enumerate([blogpost_node]):
                    content.insert(i,x)
            else:
                content.extend([blogpost_node])

            # Process any in-post .. image::s
            env.process_images(blogpost_info['docname'], blogpost_node[0].parent)

            # Process any in-post :download:s
            env.process_downloads(blogpost_info['docname'], blogpost_node[0].parent)

            # Process citations
            for ref in blogpost_node.traverse(nodes.citation_reference):
                label = ref.astext()
                if label in env.citations:
                    ref['refid'] = env.citations[label][1]

            # (Recursively) resolve references in the blogpost content
            env.resolve_references(blogpost_node, fromdocname, app.builder)

            # Find the "project" links and resolve them
            for ref in blogpost_node.traverse(nodes.reference):
                if 'project' in ref and 'refuri' in ref:
                    ref['refdocname'] = fromdocname
                    ref['refuri'] = app.builder.get_relative_uri(fromdocname, ref['project']+'/index')

        blogbody_node += content


        if 'nr_posts' in node['options'] or 'nr_days' in node['options']:
            seeall_node = nodes.container('', classes=['seeall'])
            seeall_para = nodes.paragraph('','');
            seeall_ref = nodes.reference('','')
            seeall_ref['refdocname'] = fromdocname
            seeall_ref['refuri'] = app.builder.get_relative_uri('.', 'all')
            seeall_ref.append(nodes.Text('See all status updates', 'See all status updates'))
            seeall_para += seeall_ref
            seeall_node += seeall_para
            blogbody_node += seeall_node

        node.replace_self(blogbody_node)
