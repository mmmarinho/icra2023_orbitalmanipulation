# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
from datetime import date
from marinholab.papers.icra2023.orbital_manipulation import __version__

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'marinholab-papers-icra2023-orbitalmanipulation'
copyright = f'2025-{date.today().year}, Murilo M. Marinho'
author = 'Murilo M. Marinho'
release = __version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "autodoc2",
    "sphinx.ext.viewcode"
]
autodoc2_render_plugin = "myst"
autodoc2_packages = [
    {
        "path": "../src/marinholab/papers/icra2023/orbital_manipulation",
    },
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

myst_enable_extensions = ["fieldlist", "deflist"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
# https://github.com/sphinx-extensions2/sphinx-autodoc2/blob/main/docs/conf.py
html_theme = "furo"
html_theme_options = {
    "top_of_page_button": "edit",
    "source_repository": "https://github.com/mmmarinho/icra2023_orbitalmanipulation",
    "source_branch": "main",
    "source_directory": "docs/",
    "announcement": "<em>Feedback welcomed at "
    "<a href='https://github.com/mmmarinho/icra2023_orbitalmanipulation/issues'>Issue Tracker</a></em>",
}

