# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

# sys.path.insert(0, os.path.abspath('.'))
# sys.path.append(os.path.abspath('sphinxext'))
# sys.path.insert(0, os.path.abspath('matplotlib'))


# -- Project information -----------------------------------------------------


project = "SEAHOWL"
copyright = "2022-2025, TotalEnergies-SE"
author = "Power R&D Team"

# The full version, including alpha/beta/rc tags
release = "0.11.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    #'matplotlib.sphinxext.mathmpl.math_directive',
    "matplotlib.sphinxext.mathmpl",
    "matplotlib.sphinxext.plot_directive",
    #'matplotlib.sphinxext.only_directives',
    #'matplotlib.sphinxext.plot_directive',
    #'breathe',
    "sphinx.ext.mathjax",
    "sphinx.ext.todo",
    "sphinx.ext.intersphinx",
    "sphinx.ext.inheritance_diagram",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "myst_parser",
]

autosummary_generate = True  # Turn on sphinx.ext.autosummary

# MyST parser configuration
myst_enable_extensions = [
    "colon_fence",  # ::: fence blocks
    "deflist",  # Definition lists
    "fieldlist",  # Field lists
    "tasklist",  # Checkbox lists
]

myst_heading_anchors = 3  # Auto-generate heading anchors for h1-h3

# Include TODO comments
todo_include_todos = True


# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"
# html_theme = 'haiku'

html_theme_options = {
    "canonical_url": "",
    "analytics_id": "",
    "logo_only": False,
    "display_version": True,
    "prev_next_buttons_location": "bottom",
    "style_external_links": False,
    "vcs_pageview_mode": "",
    #'style_nav_header_background': 'white',
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 5,
    "includehidden": True,
    "titles_only": False,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
# html_logo = 'totalenergies.png'
html_favicon = "favicon.ico"

# html_theme_options = {
#    'full_logo': True
# }


rst_prolog = r"""
.. |UNIT_NONE| replace:: :math:`\rm  -`

.. |UNIT_TIME| replace:: :math:`\rm s`

.. |UNIT_LENGTH| replace:: :math:`\rm m`

.. |UNIT_TEMP| replace:: K


"""
