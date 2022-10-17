import datetime
import sphinx_rtd_theme
import doctest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..")))

import torch_radon

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.mathjax',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
]

source_suffix = '.rst'
master_doc = 'index'

author = 'Matteo Ronchetti'
project = 'torch_radon'
copyright = '{}, {}'.format(datetime.datetime.now().year, author)

version = torch_radon.__version__
release = torch_radon.__version__

html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

doctest_default_flags = doctest.NORMALIZE_WHITESPACE
intersphinx_mapping = {'python': ('https://docs.python.org/', None)}
