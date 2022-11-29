# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

import os
import sys
import subprocess

sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('./erl_assignment_3/'))
sys.path.insert(0, os.path.abspath('./erl_assignment_3/scripts/'))


# -- Project information -----------------------------------------------------

project = 'ExperimentalRoboticsAssignment3'
copyright = '2022, Iacopo Pietrasanta'
author = 'Iacopo Pietrasanta'

# The full version, including alpha/beta/rc tags
release = 'v1.0.0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
# set completo
extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.napoleon',
	'sphinx.ext.autosummary',
	'sphinx.ext.doctest',
	'sphinx.ext.viewcode',
	'sphinx.ext.githubpages',
	'sphinx.ext.intersphinx',
	'sphinx.ext.todo',
	'sphinx.ext.coverage',
	'sphinx.ext.mathjax',
	'sphinx.ext.ifconfig',
	'sphinx.ext.inheritance_diagram',
	# 'sphinx.ext.duration',
	'myst_parser',
	'breathe'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for autodoc -----------------------------------------------------

# mock ros and other stuff
autodoc_mock_imports = ["rospy"]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# name of the main document (it will be the homepage of the docs)
master_doc = 'index'

# how to interpret the files
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'restructuredtext',
    '.md': 'markdown',
}


# -- Syntax Hilighting -------------------------------------------------------

# allow Pygments to guess the language
highlight_language = 'guess'


# -- Options for breathe -----------------------------------------------------

# generate the Doxygen XML documentation
subprocess.call( 'doxygen doxygen_config', shell=True )

# path of the doxygen generated HTML
breathe_projects = {
  "ExperimentalRoboticsAssignment3": "_build/xml/"
}

# title of the Doxygen project
breathe_default_project = "ExperimentalRoboticsAssignment3"
breathe_default_members = ('members', 'undoc-members')


# -- MYst Parser Settings ----------------------------------------------------

# exteensions
myst_enable_extensions = [
    "amsmath",
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    # "linkify",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist"
]

